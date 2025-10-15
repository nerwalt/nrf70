use heapless::Vec;
use crate::bindings::{
    nrf_wifi_signal,
    nrf_wifi_umac_cmd_scan,
    nrf_wifi_umac_event_new_scan_display_results,
    nrf_wifi_umac_cmd_get_scan_results,
    scan_reason,
    umac_display_results, 
    NRF_WIFI_SIGNAL_TYPE_MBM,
    NRF_WIFI_SIGNAL_TYPE_UNSPEC,
};
use embassy_time::Duration;

use crate::rpu::commands::Command;
use crate::action::Action;
use crate::util::{sliceit, unsliceit};
use crate::Error;

use crate::Control;

/// WiFi scan type.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScanType {
    /// Active scan: the station actively transmits probes that make APs respond.
    /// Faster, but uses more power.
    Active,
    /// Passive scan: the station doesn't transmit any probes, just listens for beacons.
    /// Slower, but uses less power.
    Passive,
}

/// Scan options.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ScanOptions {
    /// SSID to scan for.
    // pub ssid: Option<heapless::String<32>>,
    /// If set to `None`, all APs will be returned. If set to `Some`, only APs
    /// with the specified BSSID will be returned.
    pub bssid: Option<[u8; 6]>,
    /// Number of probes to send on each channel.
    pub nprobes: Option<u16>,
    /// Time to spend waiting on the home channel.
    pub home_time: Option<Duration>,
    /// Scan type: active or passive.
    pub scan_type: ScanType,
    /// Period of time to wait on each channel when passive scanning.
    pub dwell_time: Option<Duration>,
}

impl Default for ScanOptions {
    fn default() -> Self {
        Self {
            bssid: None,
            nprobes: None,
            home_time: None,
            scan_type: ScanType::Passive,
            dwell_time: None,
        }
    }
}

// Scan related impls
impl<'a> Control<'a> {

    /// Run a wifi scan
    pub async fn scan(&mut self, options: ScanOptions) -> Result<(), Error> {
        let mut command = nrf_wifi_umac_cmd_scan::default();

        match options.scan_type {
            ScanType::Active => {
                command.info.scan_params.passive_scan = 0;

                if let Some(dwell_time) = options.dwell_time {
                    command.info.scan_params.dwell_time_active = dwell_time.as_millis() as u16;
                }
            }
            ScanType::Passive => {
                command.info.scan_params.passive_scan = 1;

                if let Some(dwell_time) = options.dwell_time {
                    command.info.scan_params.dwell_time_passive = dwell_time.as_millis() as u16;
                }
            }
        }

        if let Some(bssid) = options.bssid {
            command.info.scan_params.mac_addr = bssid;
        }

        command.info.scan_params.num_scan_channels = 0;

        command.info.scan_reason = scan_reason::SCAN_DISPLAY as i32;

        self.action_state
            .issue(Action::Command((command.domain(), true, sliceit(&command), None)))
            .await
            .map_err(|e| {
                error!("Scan trigger failed: {:?}", e);
                e
            })?;

        self.action_state.issue(Action::WaitForScanDone).await.map_err(|e| {
            error!("Wait for scan done failed: {:?}", e);
            e
        })?;

        Ok(())
    }

    /// Get the results of the last wifi scan
    pub async fn get_scan_results(&mut self) -> Result<WifiScanResults<24>, Error> {
        let mut command = nrf_wifi_umac_cmd_get_scan_results::default();

        command.scan_reason = scan_reason::SCAN_DISPLAY as i32;

        let mut response = [0u8; 1024];

        let mut results = WifiScanResults::<24>::default();

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
                    let raw_results: &nrf_wifi_umac_event_new_scan_display_results = unsliceit(&response[..length]);
                    match WifiScanResults::try_from(raw_results) {
                        Ok(new_results) => {
                            if let Err(_) = results.extend(&new_results) {
                                return Ok(results)
                            }
                        }
                        Err(_err) => {
                            return Err(Error::InvalidData)
                        }
                    }
                }
            }
            Err(error) => {
                error!("Failed to get stats: {:?}", error);
                return Err(error);
            }
        }

        Ok(results)
    }

}

/// Wifi Access Point (AP)
#[derive(Clone, Debug)]
pub struct WifiAp {
    /// SSID 
    pub ssid: Vec<u8, 32>,
    /// BSSID (MAC address)
    pub bssid: [u8; 6],
    /// RSSI in dBm 
    pub rssi: i16,
    /// Frequency (Band) in MHz
    pub frequency: f32,
    /// Channel number
    pub channel: u32,
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiAp {
    fn format(&self, f: defmt::Formatter) {
        if let Ok(s) = str::from_utf8(&self.ssid) {
            defmt::write!(
                f,
                "WifiAp{{ssid=\"{=str}\", bssid={=[u8]:x}, rssi={=i16}dBm}}, freq={}, chan={}",
                s,
                self.bssid,
                self.rssi,
                self.frequency,
                self.channel,
            );
        } else {
            defmt::write!(
                f,
                "WifiAp{{ssid={=[u8]:x}, bssid={=[u8]:x}, rssi={=i16}dBm}}, freq={}, chan={}",
                &self.ssid[..],
                self.bssid,
                self.rssi,
                self.frequency,
                self.channel,
            );
        }
    }
}


/// Wifi Scan results
#[derive(Default)]
pub struct WifiScanResults<const N: usize = 8> {
    pub aps: Vec<WifiAp, N>,
}

impl<const N: usize> WifiScanResults<N> {

    pub fn extend(&mut self, other: &WifiScanResults) -> Result<(), ()> {
        self.aps.extend_from_slice(other.aps.as_slice()).map_err(|_| ())?;
        // for ap in other.aps {
        //     self.aps.push(ap).map_err(|_| ())?;
        // }
        Ok(())
    }
    
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiScanResults {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "WifiScanResults{{count={=usize}}}", self.aps.len());
    }
}

impl From<&umac_display_results> for WifiAp {
    fn from(r: &umac_display_results) -> Self {
        // SSID
        let ssid_len = core::cmp::min(r.ssid.nrf_wifi_ssid_len as usize, 32);
        let mut ssid: Vec<u8, 32> = Vec::new();
        let _ = ssid.extend_from_slice(&r.ssid.nrf_wifi_ssid[..ssid_len]);

        // BSSID
        let mut bssid = [0u8; 6];
        bssid.copy_from_slice(&r.mac_addr);

        // RSSI
        let rssi = rssi_dbm_from_signal(&r.signal);

        // Frequency (band)
        let frequency = match r.nwk_band {
            // nrf_wifi_band::NRF_WIFI_BAND_2GHZ
            0 => 2.4,
            // nrf_wifi_band::NRF_WIFI_BAND_5GHZ
            1 => 5.0,
            // nrf_wifi_band::NRF_WIFI_BAND_60GHZ
            2 => 60.0,
            // nrf_wifi_band::NRF_WIFI_BAND_INVALID
            _ => f32::NAN,
        };

        // Network channel
        let channel  = r.nwk_channel;

        WifiAp { ssid, bssid, rssi, frequency, channel }
    }
}

impl From<&nrf_wifi_umac_event_new_scan_display_results> for WifiScanResults {
    fn from(ev: &nrf_wifi_umac_event_new_scan_display_results) -> Self {
        let count = core::cmp::min(ev.event_bss_count as usize, 8);
        let mut aps: Vec<WifiAp, 8> = Vec::new();
        for i in 0..count {
            let ap = WifiAp::from(&ev.display_results[i]);
            // The push should not fail, so ignore the result
            let _ = aps.push(ap);
        }
        WifiScanResults { aps }
    }
}

/// Convert nrf_wifi_signal signal into dBm.
fn rssi_dbm_from_signal(sig: &nrf_wifi_signal) -> i16 {
    let typ = sig.signal_type as u32;

    if typ == NRF_WIFI_SIGNAL_TYPE_MBM {
        let mbm = unsafe { sig.signal.mbm_signal } as i32;
        // Convert mBm -> dBm, clamp to i16.
        (mbm / 100).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    } else if typ == NRF_WIFI_SIGNAL_TYPE_UNSPEC {
        let unspec = unsafe { sig.signal.unspec_signal } as i16;
        // Map 0..100 -> approx -100..0 dBm
        unspec.saturating_sub(100)
    } else  {
        0
    }
}
