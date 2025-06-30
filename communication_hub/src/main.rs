use std::sync::Arc;
use tokio::sync::Mutex;
use tonic::{transport::Server, Request, Response, Status};
use tokio_modbus::client::tcp;
use tokio_modbus::prelude::*;
use tokio_modbus::client::Context;  

use tokio::time::sleep;                
use std::time::Duration;               

/* ToDo:
- Handle runtime connection lost with the delivery hub
*/


pub mod rpc {
    tonic::include_proto!("robot_to_del");
}

#[derive(Clone)]
pub struct MyPackageStatusService {
    // our shared Modbus context
    client: Arc<Mutex<Context>>,       // ← public type
}

#[tonic::async_trait]
impl rpc::package_status_server::PackageStatus for MyPackageStatusService {
    async fn robot_to_server(
        &self,
        request: Request<rpc::PackageAcceptanceReq>,
    ) -> Result<Response<rpc::PackageAcceptanceResp>, Status> {
        let req = request.into_inner();

        // lock the Modbus client for this call
        let mut ctx = self.client.lock().await;
        // e.g. read register 0
        let regs = ctx
            .read_holding_registers(0, 1)
            .await
            .map_err(|e| Status::internal(format!("Modbus error: {}", e)))?;

        println!(
            "Robot {} @hub {} slot {:?} → got regs={:?}",
            req.robot_id,
            req.del_hub_id,
            std::str::from_utf8(&req.slot_id).ok(),
            regs
        );

        // your acceptance logic
        let accepted = true;
        Ok(Response::new(rpc::PackageAcceptanceResp { accepted }))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // connect once
    let socket_addr = "127.0.0.1:5020".parse().unwrap();

    /*wait for the connection with delivery hub
    In case its not available, it will retry every second
    until it connects successfully.
    */
    let ctx: Context = loop {
        match tcp::connect(socket_addr).await {
            Ok(ctx) => {
                println!("Modbus connected!");
                break ctx;         // ← here you “break” the loop with a value of type `Context`
            }
            Err(e) => {
                eprintln!("Failed to connect… retrying in 1s: {:?}", e);
                sleep(Duration::from_secs(1)).await;
                // no break here, so the loop continues
            }
        }
    };

    let shared = Arc::new(Mutex::new(ctx));

    let svc = MyPackageStatusService { client: shared.clone() };
    let addr = "[::1]:50051".parse()?;
    println!("Server listening on {}", addr);

    Server::builder()
        .add_service(rpc::package_status_server::PackageStatusServer::new(svc))
        .serve(addr)
        .await?;

    Ok(())
}
