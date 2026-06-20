#![no_std]

use embedded_io_async::{Read, ReadExactError};

#[derive(Debug, Default, Clone, Copy)]
pub struct LidarData
{
    pub mm_dist: f32,
    pub angle_dg: f32,
    pub quality: u8,
    pub start_flag: bool,
}

#[derive(Debug)]
pub enum ErrorCasesLidar<E> 
{
    UartReadError(ReadExactError<E>),
    SyncError,
}

pub struct LidarS3<UART>
{
    uart: UART
}

impl<UART> LidarS3<UART>
where UART: Read,
{
    pub fn new(uart: UART) -> Self
    {
        Self { uart}
    }
    pub async fn read_new_data(&mut self) -> Result<LidarData, ErrorCasesLidar<UART::Error>>
    {
        let mut buff = [0u8; 1];
        loop 
        {
            self.uart.read_exact(&mut buff).await.map_err(ErrorCasesLidar::UartReadError)?;
            let start_flag = (buff[0] & 0x01) == 1;
            let inverse_start = ((buff[0] >> 1) & 0x01) == 1;
            let b0 = buff[0];
            if start_flag != inverse_start
            {
                self.uart.read_exact(&mut buff).await.map_err(ErrorCasesLidar::UartReadError)?;
                if (buff[0] & 0x01) == 1
                {
                    let mut rest = [0u8; 3];
                    self.uart.read_exact(&mut rest).await.map_err(ErrorCasesLidar::UartReadError)?;
                    let packet = [b0, buff[0], rest[0], rest[1], rest[2]];
                    return self.parser_lidar(&packet);
                }
            }
        }
    }

    fn parser_lidar(&self, packet: &[u8; 5]) -> Result<LidarData, ErrorCasesLidar<UART::Error>>
    {
        let start_flag = (packet[0] & 0x01) == 1;
        let quality = packet[0] >> 2;
        let angle_raw = ((packet[2] as u16) << 8) | (packet[1] as u16);
        let angle_dg = (angle_raw >> 1) as f32 / 64.0;
        let dist_raw = ((packet[4] as u16) << 8) | (packet[3] as u16);
        let mm_dist = dist_raw as f32 / 4.0;
        Ok(LidarData {
            mm_dist,
            angle_dg,
            quality,
            start_flag,
        })
    }
}

pub struct LidarMap
{
    pub points: [LidarData; 360],
}
impl LidarMap
{
    pub fn new() -> Self
    {
        Self
        {
            points: [LidarData::default(); 360],
        }
    }


    pub fn update_points(&mut self, data: LidarData)
    {
        let index = ((data.angle_dg + 0.5) as usize) % 360;
        if data.quality > self.points[index].quality
        {
            self.points[index] = data;
        }

    }
    pub fn get_dist(&self, angle: usize) -> f32
    {
        let index = angle % 360;
        self.points[index].mm_dist
    }
}

