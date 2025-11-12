#![no_std]
#![deny(unused_must_use)]
#![allow(async_fn_in_trait)]
#![warn(clippy::all, clippy::pedantic, clippy::cargo)]

pub(crate) mod fmt;

use core::mem::{transmute, transmute_copy};

use action::{Action, ActionState, Item};
use bindings::*;
use bus::Bus;
use embassy_futures::select::{Either3, select3};
use embassy_net_driver_channel as ch;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use fmt::Bytes;
use net::{NetworkBuffer, eth};
use rpu::Rpu;
use rpu::firmware::{FirmwareInfo, FirmwareParseError};
use rpu::memory::regions::*;
use util::{meh, slice8, sliceit, unsliceit, unsliceit2};

mod action;
pub mod bus;
pub mod control;
mod net;
mod rpu;
mod util;

#[allow(dead_code)]
mod bindings;

// TODO FIXME TEMPORARY
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::bindings::nrf_wifi_umac_event_new_scan_display_results;
pub type ScanResultsChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, nrf_wifi_umac_event_new_scan_display_results, 10>;
pub static SCANE_RESULTS_CHANNEL: ScanResultsChannel = ScanResultsChannel::new();

const MTU: usize = 1514;

// const SR0_WRITE_IN_PROGRESS: u8 = 0x01;
const SR1_RPU_AWAKE: u8 = 0x02;
const SR1_RPU_READY: u8 = 0x04;
const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NoAcknowledgement,
    Timeout,
    InvalidAddress,
    InvalidArgument,
    InvalidData,
    NotInitialized,
    BufferTooSmall,
    BufferOverflow,
    NoData,
    NotFound,
    NotHandled(u32),
    Busy,
    FirmwareParseError(FirmwareParseError),
    Code(i32),
}

pub struct State {
    action_state: ActionState,
    ch: ch::State<MTU, 4, 4>,
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl State {
    #[must_use]
    pub fn new() -> Self {
        Self {
            ch: ch::State::new(),
            action_state: ActionState::new(),
        }
    }
}

#[allow(dead_code)]
pub struct Control<'a> {
    action_state: &'a ActionState,
    state_ch: ch::StateRunner<'a>,
}

pub type NetDriver<'a> = ch::Device<'a, MTU>;

#[allow(dead_code)]
pub struct Runner<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> {
    ch: ch::Runner<'a, MTU>,
    state_ch: ch::StateRunner<'a>,
    action_state: &'a ActionState,

    rpu: Rpu<BUS>,
    bucken: OUT,
    iovdd_ctl: OUT,
    host_irq: IN,
}

pub async fn new<'a, BUS, IN, OUT>(
    state: &'a mut State,
    bus: BUS,
    bucken: OUT,
    iovdd_ctl: OUT,
    host_irq: IN,
) -> (NetDriver<'a>, Control<'a>, Runner<'a, BUS, IN, OUT>)
where
    BUS: Bus,
    IN: InputPin + Wait,
    OUT: OutputPin,
{
    let (ch_runner, device) = ch::new(&mut state.ch, ch::driver::HardwareAddress::Ethernet([0; 6]));
    let state_ch = ch_runner.state_runner();

    let mut runner = Runner {
        ch: ch_runner,
        state_ch,
        action_state: &state.action_state,
        rpu: Rpu::new(bus),
        bucken,
        iovdd_ctl,
        host_irq,
    };
    runner.init().await;

    let control = Control {
        action_state: &state.action_state,
        state_ch,
    };

    (device, control, runner)
}

impl<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'a, BUS, IN, OUT> {
    async fn init(&mut self) {
        Timer::after(Duration::from_millis(10)).await;
        self.bucken.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;
        self.iovdd_ctl.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;
    }

    pub async fn run(&mut self) -> ! {
        let mut buffer_u32 = [0u32; (MAX_EVENT_POOL_LEN / 4) as usize];

        loop {
            let action = self.action_state.wait_pending();
            let wifi_tx = self.ch.tx_buf();
            let irq_event = self.host_irq.wait_for_high();

            match select3(action, wifi_tx, irq_event).await {
                Either3::First(action) => {
                    debug!("Action: {:?}", action);

                    match action {
                        Action::Boot(firmware) => match self.boot(firmware).await {
                            Ok(()) => (),
                            Err(error) => self.action_state.respond(Err(error)),
                        },
                        Action::Command((kind, wait_for_completion, buffer, _)) => {
                            match self.rpu.send_command_raw(kind, buffer).await {
                                Ok(()) => {
                                    if !wait_for_completion {
                                        self.action_state.respond(Ok(None));
                                    }
                                }
                                Err(error) => self.action_state.respond(Err(error)),
                            }
                        }
                        Action::Get((item, _)) => match item {
                            Item::UmacInfo => {
                                let umac_info = self.rpu.retrieve_umac_info().await;
                                let umac_info_buffer = sliceit(&umac_info);

                                self.action_state.respond(Ok(Some(&umac_info_buffer[..])));
                            }
                        },
                        Action::WaitForDone => { },
                    };
                }
                Either3::Second(packet) => {
                    debug!("tx pkt {:02x}", Bytes(&packet[..packet.len().min(48)]));
                }
                Either3::Third(irq) => {
                    debug!("Got IRQ, draining event queue...");

                    match irq {
                        Ok(()) => self.rpu.irq_ack().await,
                        Err(_) => continue,
                    }

                    self.drain_event_queue(&mut buffer_u32).await;

                    if self.rpu.irq_watchdog_check().await {
                        self.rpu.irq_watchdog_ack().await;
                    }
                }
            }
        }
    }

    async fn boot(&mut self, firmware: *const [u8]) -> Result<(), Error> {
        let firmware_info = FirmwareInfo::read(firmware)?;
        self.rpu.boot(&firmware_info).await
    }

    async fn drain_event_queue(&mut self, buffer_u32: &mut [u32; 250]) {
        loop {
            match self.rpu.read_event(buffer_u32).await {
                Ok(message) => {
                    let message_type = message.type_ as u32;
                    let message_size = message.hdr.len as usize;

                    let message_type = match nrf_wifi_host_rpu_msg_type::try_from(message_type) {
                        Ok(message_type) => {
                            debug!(
                                "Got {:?} event ({}). Message length: {}",
                                message_type, message_type as u32, message_size
                            );
                            Some(message_type)
                        }
                        Err(_) => {
                            warn!("Unknown event type {:08x}", message_type);
                            None
                        }
                    };

                    let buffer_u8 = slice8(buffer_u32);

                    if let Some(message_type) = message_type {
                        match message_type {
                            nrf_wifi_host_rpu_msg_type::NRF_WIFI_HOST_RPU_MSG_TYPE_SYSTEM => {
                                self.handle_system_message(buffer_u8, message_size);
                            }
                            nrf_wifi_host_rpu_msg_type::NRF_WIFI_HOST_RPU_MSG_TYPE_UMAC => {
                                self.handle_umac_message(buffer_u8, message_size);
                            }
                            nrf_wifi_host_rpu_msg_type::NRF_WIFI_HOST_RPU_MSG_TYPE_DATA => {
                                match self.handle_data_message(buffer_u8).await {
                                    Ok(()) => {}
                                    Err(err) => warn!("Failed to handle data message {:?}", err),
                                }
                            }
                            nrf_wifi_host_rpu_msg_type::NRF_WIFI_HOST_RPU_MSG_TYPE_SUPPLICANT => {
                                debug!("Got supplicant event, ignoring...");
                            }
                        }
                    } else {
                        warn!("Unhandled message type: {}", meh(message.type_));
                    }
                }
                Err(_) => break,
            }
        }
    }

    fn handle_system_message(&self, buffer: &[u8], size: usize) {
        let header: &nrf_wifi_sys_head = unsliceit(buffer);
        let event = nrf_wifi_sys_events::try_from(header.cmd_event as u32);
        let payload_length = header.len;

        if let Ok(event) = event {
            debug!("Processing system event: {:?}, length: {}", event, payload_length);
        }

        match event {
            Ok(nrf_wifi_sys_events::NRF_WIFI_EVENT_INIT_DONE) => {
                self.action_state.respond(Ok(None));
            }
            Ok(nrf_wifi_sys_events::NRF_WIFI_EVENT_STATS) => self.action_state.respond(Ok(Some(&buffer[..size]))),
            _ => warn!("System event not handled: {:08x}", meh(header.cmd_event)),
        }
    }

    fn handle_umac_message(&mut self, buffer: &[u8], size: usize) {
        let header: &nrf_wifi_umac_hdr = unsliceit(buffer);
        let event = nrf_wifi_umac_events::try_from(header.cmd_evnt as u32);
        let header_length = size_of::<nrf_wifi_umac_hdr>();
        let payload_length = size - header_length;

        if let Ok(event) = event {
            debug!(
                "Processing UMAC event: {:?} ({}), length: {}",
                event, event as u32, payload_length
            );
        }

        match event {
            Ok(nrf_wifi_umac_events::NRF_WIFI_UMAC_EVENT_CMD_STATUS) => {
                let command: &nrf_wifi_umac_event_cmd_status = unsliceit(buffer);
                let status = command.cmd_status as i32;

                let command_type = nrf_wifi_umac_commands::try_from(meh(command.cmd_id));

                if let Ok(command_type) = command_type {
                    debug!(
                        "Command {:?} ({}) finished with status {}",
                        command_type,
                        meh(command.cmd_id),
                        status
                    );
                } else {
                    debug!(
                        "Command UNKNOWN ({}) finished with status {}",
                        meh(command.cmd_id),
                        status
                    );
                }

                match status {
                    0 => self.action_state.respond(Ok(None)),
                    error => self.action_state.respond(Err(Error::Code(error as i32))),
                }
            }
            Ok(nrf_wifi_umac_events::NRF_WIFI_UMAC_EVENT_IFFLAGS_STATUS) => {
                let state: &nrf_wifi_umac_event_vif_state = unsliceit(buffer);
                let status = state.status;

                debug!("Interface flags update finished with status {}", status);

                match status {
                    0 => self.action_state.respond(Ok(None)),
                    error => self.action_state.respond(Err(Error::Code(error))),
                }
            }
            Ok(nrf_wifi_umac_events::NRF_WIFI_UMAC_EVENT_TRIGGER_SCAN_START) => {
                let response: &nrf_wifi_umac_event_trigger_scan = unsliceit(buffer);

                debug!(
                    "Scan started. Flags {:#x}. SSIDs: {}. Frequencies: {}. IE length: {}.",
                    meh(response.nrf_wifi_scan_flags),
                    meh(response.num_scan_ssid),
                    meh(response.num_scan_frequencies),
                    meh(response.ie.ie_len)
                );
            }
            Ok(nrf_wifi_umac_events::NRF_WIFI_UMAC_EVENT_SCAN_DISPLAY_RESULT) => {
                let event: &nrf_wifi_umac_event_new_scan_display_results = unsliceit(buffer);
                if let Err(_err) = SCANE_RESULTS_CHANNEL.try_send(*event) {
                    error!("Error sending scan results on channel");
                    self.action_state.respond(Err(Error::BufferTooSmall));
                }
                info!(">>> sent scan results on channel {}", SCANE_RESULTS_CHANNEL.len());
                if event.umac_hdr.seq != 0 {
                    info!(">>> more scan results");
                    // self.action_state.respond_chunk(Ok((Some(buffer as *const [u8]), true)));
                } else {
                    info!(">>> scan results done");
                    self.action_state.respond(Ok(None));
                }
                // self.action_state.respond(Ok(Some(buffer as *const [u8])));
            }
            Ok(nrf_wifi_umac_events::NRF_WIFI_UMAC_EVENT_SCAN_DONE) => {
                let event: &nrf_wifi_umac_event_scan_done = unsliceit(buffer);

                let result = if event.status == 0 {
                    Ok(None)
                } else {
                    Err(Error::Code(event.status))
                };

                self.action_state.respond(result);
            }
            _ => warn!("UMAC event not handled: {:#08x}", meh(header.cmd_evnt)),
        }
    }

    async fn handle_data_message(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let header: &nrf_wifi_umac_head = unsliceit(buffer);
        let command = nrf_wifi_umac_data_commands::try_from(header.cmd);

        if let Ok(command) = command {
            debug!("Processing DATA command: {:?}, length: {}", command, meh(header.len));
        }

        match command {
            Ok(nrf_wifi_umac_data_commands::NRF_WIFI_CMD_CARRIER_ON) => {
                let carrier_state: &nrf_wifi_data_carrier_state = unsliceit(buffer);
                debug!("Carrier state ON for WDEV {}", meh(carrier_state.wdev_id));
                Ok(())
            }
            Ok(nrf_wifi_umac_data_commands::NRF_WIFI_CMD_CARRIER_OFF) => {
                let carrier_state: &nrf_wifi_data_carrier_state = unsliceit(buffer);
                debug!("Carrier state OFF for WDEV {}", meh(carrier_state.wdev_id));
                Ok(())
            }
            Ok(nrf_wifi_umac_data_commands::NRF_WIFI_CMD_RX_BUFF) => self.handle_rx_buffer(buffer).await,
            _ => Err(Error::NotHandled(meh(header.cmd))),
        }
    }

    async fn handle_rx_buffer(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let (rx_packet, buf) = unsliceit2::<nrf_wifi_rx_buff>(buffer);

        let rx_packet_type = nrf_wifi_rx_pkt_type::try_from(meh(rx_packet.rx_pkt_type) as u32);

        let number_of_packets = rx_packet.rx_pkt_cnt as usize;
        let mac_header_length = rx_packet.mac_header_len as usize;

        debug!(
            "Got RX buffer. # packets: {}. Frequency: {}",
            number_of_packets,
            meh(rx_packet.frequency)
        );

        let rx_buffer_infos: &[nrf_wifi_rx_buff_info] = unsafe { transmute(buf) };

        for packet_index in 0..number_of_packets {
            let rx_buffer_info = rx_buffer_infos[packet_index];

            let packet_descriptor_identifier = rx_buffer_info.descriptor_id as usize;
            let packet_length = rx_buffer_info.rx_pkt_len as usize;
            let packet_type = rx_buffer_info.pkt_type as u32;

            debug!(
                "RX packet - Descriptor: {}. Length: {}. Type: {}",
                packet_descriptor_identifier, packet_length, packet_type
            );

            self.rpu
                .update_cached_receive_buffer(packet_descriptor_identifier, packet_length)
                .await?;

            let raw_buffer = self.rpu.get_cached_receive_buffer_slice(packet_descriptor_identifier)?;

            let mut network_buffer = NetworkBuffer::new(raw_buffer, packet_length);

            match rx_packet_type {
                Ok(nrf_wifi_rx_pkt_type::NRF_WIFI_RX_PKT_DATA) => {
                    match packet_type {
                        PKT_TYPE_MPDU => {
                            let header: nrf_wifi_fmac_ieee80211_hdr = unsafe {
                                let mut header_buffer = [0u8; size_of::<nrf_wifi_fmac_ieee80211_hdr>()];
                                header_buffer.copy_from_slice(
                                    &network_buffer.get_data()[..size_of::<nrf_wifi_fmac_ieee80211_hdr>()],
                                );

                                transmute_copy(&header_buffer)
                            };

                            let eth_type_buffer: [u8; 2] = network_buffer.get_data()
                                [mac_header_length + 6..mac_header_length + 8]
                                .try_into()
                                .map_err(|_| Error::BufferTooSmall)?;

                            let eth_type = eth::get_type(&eth_type_buffer);
                            let header_size = mac_header_length + eth::get_skip_header_bytes(eth_type);

                            const ETH_HEADER_SIZE: usize = size_of::<nrf_wifi_fmac_eth_hdr>();

                            // Skip to the ETH header
                            network_buffer.increase_head_room(header_size - ETH_HEADER_SIZE)?;

                            let eth_header_ptr = network_buffer.get_data().as_ptr() as *mut nrf_wifi_fmac_eth_hdr;
                            let data_length = network_buffer.get_data().len() - ETH_HEADER_SIZE;

                            unsafe {
                                eth_header_ptr.write(nrf_wifi_fmac_eth_hdr::new(data_length as u16, &header, eth_type));
                            }

                            let payload = network_buffer.get_data();

                            match self.ch.try_rx_buf() {
                                Some(buf) => {
                                    debug!("Copying {} bytes into buffer", payload.len());
                                    buf[..payload.len()].copy_from_slice(payload);
                                    self.ch.rx_done(payload.len())
                                }
                                None => warn!("failed to push RX packet to the channel."),
                            }
                        }
                        PKT_TYPE_MSDU_WITH_MAC => {
                            warn!("PKT_TYPE_MSDU_WITH_MAC is unhandled");
                        }
                        PKT_TYPE_MSDU => {
                            warn!("PKT_TYPE_MSDU is unhandled");
                        }
                        _ => warn!("Unknown packet type {}", packet_type),
                    }
                }
                Ok(nrf_wifi_rx_pkt_type::NRF_WIFI_RX_PKT_BCN_PRB_RSP) => {
                    // let mut buffer: heapless::String<512> = heapless::String::new();
                    // util::hexdump(&mut buffer, network_buffer.get_data());
                    // info!("{}", buffer);
                }
                _ => {
                    let rx_packet_type = rx_packet.rx_pkt_type;

                    warn!("Unknown RX packet type: {:#x}", rx_packet_type);
                    return Err(Error::NotHandled(rx_packet_type as u32));
                }
            }

            self.rpu.release_rx_buffer(packet_descriptor_identifier).await?;
        }

        Ok(())
    }
}
