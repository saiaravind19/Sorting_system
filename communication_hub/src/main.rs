/*use std::net::SocketAddr;
use tokio_modbus::prelude::*;
use tokio_modbus::client::tcp;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Parse the TCP socket address of your Modbus server
    let socket_addr: SocketAddr = "127.0.0.1:5020".parse()?;

    // 2. Establish an async connection
    let mut ctx = tcp::connect(socket_addr).await?;
    println!("Connected to Modbus TCP at {}", socket_addr);

    // 3. Read holding registers starting at address 0x10, length 5
    let regs = ctx.read_holding_registers(0,1).await?;
    println!("Holding registers @0x10: {:?}", regs);


    // 6. Cleanly close the connection
    ctx.disconnect().await?;
    println!("Disconnected");

    Ok(())
}


// SPDX-FileCopyrightText: Copyright (c) 2017-2025 slowtec GmbH <post@slowtec.de>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Synchronous TCP client example

fn main() {
    use tokio_modbus::prelude::*;

    let socket_addr = "127.0.0.1:5020".parse().unwrap();
    let mut ctx = sync::tcp::connect(socket_addr).unwrap();
    let buff = ctx.read_holding_registers(0,1).unwrap();
    println!("Response is '{buff:?}'");
}
*/
// SPDX-FileCopyrightText: Copyright (c) 2017-2025 slowtec GmbH <post@slowtec.de>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Asynchronous TCP client example

use tokio_modbus::prelude::*;
use tokio_modbus::client::tcp;


#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    let socket_addr = "127.0.0.1:5020".parse().unwrap();

    let mut ctx = tcp::connect(socket_addr).await?;

    println!("Fetching the coupler ID");
    let data = ctx.read_holding_registers(0, 1).await?;

    println!("Disconnecting {:?}",data);
    ctx.disconnect().await?;

    Ok(())
}

