use core::ptr;

use crate::action::{Action, Item};
use crate::bindings::{
    host_rpu_umac_info, nrf_wifi_cmd_get_stats, 
    nrf_wifi_sys_umac_event_stats, nrf_wifi_umac_change_macaddr_info, nrf_wifi_umac_cmd_change_macaddr,
    nrf_wifi_umac_cmd_chg_vif_state, 
     nrf_wifi_umac_hdr, 
};
use crate::rpu::commands::Command;
use crate::util::sliceit;
use crate::Error;

use crate::Control;

pub mod scan;

#[allow(dead_code)]
impl<'a> Control<'a> {
    pub async fn init(&mut self, firmware: &'static [u8]) -> Result<(), Error> {
        self.action_state.issue(Action::Boot(firmware)).await.map(|_| ())?;
        info!("Boot done");

        // --- Update MAC address ---

        let mut umac_info_buffer = [0u8; size_of::<host_rpu_umac_info>()];

        self.action_state
            .issue(Action::Get((Item::UmacInfo, &mut umac_info_buffer[..])))
            .await?;

        let umac_info: host_rpu_umac_info = unsafe { core::mem::transmute_copy(&umac_info_buffer) };

        let mac_address = [
            (umac_info.mac_address0[0]) as u8,
            (umac_info.mac_address0[0] >> 8) as u8,
            (umac_info.mac_address0[0] >> 16) as u8,
            0x0,
            (umac_info.mac_address0[1]) as u8,
            (umac_info.mac_address0[1] >> 8) as u8,
        ];

        let header = nrf_wifi_umac_hdr::default();
        let mut command = nrf_wifi_umac_cmd_change_macaddr {
            umac_hdr: header,
            macaddr_info: nrf_wifi_umac_change_macaddr_info { mac_addr: mac_address },
        };
        command.prepare();

        match self
            .action_state
            .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
            .await
        {
            Ok(_) => {}
            Err(error) => {
                error!("Failed to set MAC address: {:?}", error);
                return Err(error);
            }
        }

        info!(
            "Set MAC address for interface: {:#x}:{:#x}:{:#x}:{:#x}:{:#x}:{:#x}",
            mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]
        );

        // --- Bring interface up ---

        let mut command = nrf_wifi_umac_cmd_chg_vif_state::default();
        command.info.state = 1;
        command.prepare();

        match self
            .action_state
            .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
            .await
        {
            Ok(_) => {}
            Err(error) => {
                error!("Failed to change interface state: {:?}", error);
                return Err(error);
            }
        }

        info!("Brought interface up");

        /*
        // let result = self.read_u32_from_region(SYSBUS, 0x0C0).await;
        // info!("PART: {}", result);

        // TODO: pass in as a config?
        const POWER_SAVE_ENABLED: bool = false;

        let mut command = nrf_wifi_umac_cmd_set_power_save {
            umac_hdr: nrf_wifi_umac_hdr::default(),
            info: nrf_wifi_umac_set_power_save_info {
                ps_state: if POWER_SAVE_ENABLED {
                    nrf_wifi_ps_state::NRF_WIFI_PS_ENABLED as i32
                } else {
                    nrf_wifi_ps_state::NRF_WIFI_PS_DISABLED as i32
                },
            },
        };
        command.prepare();

        match self
            .action_state
            .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
            .await
        {
            Ok(_) => {}
            Err(err) => {
                error!("Failed to set power save: {:?}", err);
                return Err(err);
            }
        }

        // --- Set mcast address ---

        /*
                TODO: should this be set another way?
                static const struct in_addr all_systems = { { { 224, 0, 0, 1 } } };
                static const struct in_addr all_routers = { { { 224, 0, 0, 2 } } };
                mac_addr->addr[0] = 0x01;
                mac_addr->addr[1] = 0x00;
                mac_addr->addr[2] = 0x5e;
                mac_addr->addr[3] = ipv4_addr->s4_addr[1];
                mac_addr->addr[4] = ipv4_addr->s4_addr[2];
                mac_addr->addr[5] = ipv4_addr->s4_addr[3];

                mac_addr->addr[3] &= 0x7f;
        */
        let multicast_mac_address = [0x01, 0x00, 0x5e, 0x00, 0x00, 0x01];

        let mut command = nrf_wifi_umac_cmd_mcast_filter {
            umac_hdr: nrf_wifi_umac_hdr::default(),
            info: nrf_wifi_umac_mcast_cfg {
                type_: 0,
                mac_addr: multicast_mac_address,
            },
        };
        command.prepare();

        match self
            .action_state
            .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
            .await
        {
            Ok(_) => {}
            Err(err) => {
                error!("Failed to set multicast address: {:?}", err);
                return Err(err);
            }
        };

        // --- Register frame ---

        /* WNM - BSS Transition Management Request */
        /* Radio Measurement - Neighbor Report Response */
        /* Radio Measurement - Radio Measurement Request */
        let frames: [[u8; 2]; 3] = [[0x0a, 0x07], [0x05, 0x05], [0x05, 0x00]];
        const WLAN_FC_TYPE_MGMT: u16 = 0;
        const WLAN_FC_TYPE_CTRL: u16 = 1;
        const WLAN_FC_TYPE_DATA: u16 = 2;

        const WLAN_FC_STYPE_ACTION: u16 = 13;

        for frame in frames {
            let mut command = nrf_wifi_umac_cmd_mgmt_frame_reg {
                umac_hdr: nrf_wifi_umac_hdr::default(),
                info: nrf_wifi_umac_mgmt_frame_info {
                    frame_type: (WLAN_FC_TYPE_MGMT << 2) | (WLAN_FC_STYPE_ACTION << 4),
                    frame_match: nrf_wifi_umac_frame_match {
                        frame_match_len: frame.len() as u32,
                        frame_match: unsafe { zeroed() },
                    },
                },
            };

            command.info.frame_match.frame_match[0..frame.len()].copy_from_slice(&frame);

            command.prepare();

            match self
                .action_state
                .issue(Action::Command((command.domain(), false, sliceit(&command), None)))
                .await
            {
                Ok(_) => {}
                Err(err) => {
                    error!("Failed to register frame: {:?}", err);
                    return Err(err);
                }
            };
        }
        */

        // --- Get wiphy ---
        //
        // TODO: need to handle large messages for this
        //
        // let mut command = nrf_wifi_cmd_get_wiphy {
        //     umac_hdr: nrf_wifi_umac_hdr::default(),
        // };
        // command.prepare();
        //
        // match self
        //     .action_state
        //     .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
        //     .await
        // {
        //     Ok(_) => {}
        //     Err(err) => {
        //         error!("Failed to register frame: {:?}", err);
        //         return Err(err);
        //     }
        // };

        // --- Delete 6 keys ---
        //
        // TOOD: Unsure of which here or if this is just clearing some keys
        // Command 7

        // --- Update station entry ---
        // let mut command = nrf_wifi_umac_cmd_chg_sta {
        //     umac_hdr: nrf_wifi_umac_hdr::default(),
        //     valid_fields: NRF_WIFI_CMD_SET_STATION_STA_FLAGS2_VALID,
        //     info: unsafe { zeroed() },
        // };

        // command.prepare();

        // match self
        //     .action_state
        //     .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
        //     .await
        // {
        //     Ok(_) => {}
        //     Err(err) => {
        //         error!("Failed to update station entry: {:?}", err);
        //         return Err(err);
        //     }
        // };

        Ok(())
    }

    pub async fn get_stats(&mut self) -> Result<(), Error> {
        let command = nrf_wifi_cmd_get_stats::default();

        let mut response = [0u8; 1024];

        match self
            .action_state
            .issue(Action::Command((
                command.domain(),
                true,
                sliceit(&command),
                Some(&mut response[..]),
            )))
            .await
        {
            Ok(response_length) => {
                if let Some(length) = response_length {
                    let data: nrf_wifi_sys_umac_event_stats = unsafe { ptr::read(response.as_ptr() as *const _) };
                    info!("{}: {:?}", length, data);
                }

                Ok(())
            }
            Err(error) => {
                error!("Failed to get stats: {:?}", error);
                Err(error)
            }
        }
    }
}
