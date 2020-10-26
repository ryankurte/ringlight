use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = 0xFF00, usage = 0x01, report_id = 0x00) = {
        (usage_page = 0xFF00,) = {
            #[item_settings data,array,absolute] command=output;
        };
    }
)]
#[allow(dead_code)]
pub struct DeviceReport {
    pub command: [u8; 32],
}
