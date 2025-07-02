use std::sync::Arc;
use std::time::Duration;

use tokio::sync::Mutex;
use tokio::time::sleep;
use tonic::{transport::Server, Request, Response, Status};

use tokio_modbus::client::tcp;
use tokio_modbus::client::Context;
use tokio_modbus::prelude::*;


/*
Add a better logic to handle package acceptance 

- Say if multiple robots are dumping one by one then the current logic will always return true
- We need to check if package is received by the hub at a particular slot.


##########

Currently just check if there is any coil as high

*/

const NUM_BINS: u16 = 6;


pub mod rpc {
    tonic::include_proto!("robot_to_commhub");
}

#[derive(Clone)]
pub struct SharedModbus {
    client: Arc<Mutex<Context>>,
}

#[tonic::async_trait]
impl rpc::delivery_hub_status_server::DeliveryHubStatus for SharedModbus 
{
    async fn robot_to_comm_hub(
        &self,
        request: Request<rpc::DeliveryHubAvailablityReq>,
    ) -> Result<Response<rpc::DeliveryHubAvailablityResp>, Status> {
        let req = request.into_inner();

        // get the hub_id and split it and get the last element and convert to int
        let id_str = req.del_hub_id.split('_').last().ok_or_else(|| 
            {
                Status::invalid_argument(format!("invalid hub_id: {}", req.del_hub_id))
            })?;  

        let slave_id = id_str.parse::<u8>().map_err(|e| Status::invalid_argument
            (format!("cannot parse slave id: {}", e)))?;

        // grab the modbus context
        let mut ctx = self.client.lock().await;
        // set the slave id to communicate with del hub
        ctx.set_slave(Slave(slave_id));

    
        let coils = ctx
            .read_coils(0, NUM_BINS)
            .await
            .map_err(|e| Status::internal(format!("Modbus error: {}", e)))?;

        // check if any coil is low - > Empty        
        let accepted = coils.into_iter().any(|bit| !bit);

        println!("[Delivery Hub Status] hub Id:{} Availability={}",req.del_hub_id, accepted);

        Ok(Response::new(rpc::DeliveryHubAvailablityResp {
            status: accepted,
        }))    
    }
}

#[tonic::async_trait]
impl rpc::package_status_server::PackageStatus for SharedModbus 
{
    async fn robot_to_server(
        &self,
        request: Request<rpc::PackageAcceptanceReq>,
    ) -> Result<Response<rpc::PackageAcceptanceResp>, Status> {
        let req = request.into_inner();

        // get the hub_id and split it and get the last element and convert to int
        let id_str = req.del_hub_id.split('_').last().ok_or_else(|| 
            {
                Status::invalid_argument(format!("invalid hub_id: {}", req.del_hub_id))
            })?;  

        let slave_id = id_str.parse::<u8>().map_err(|e| Status::invalid_argument
            (format!("cannot parse slave id: {}", e)))?;

        // grab the modbus context
        let mut ctx = self.client.lock().await;
        // set the slave id to communicate with del hub
        ctx.set_slave(Slave(slave_id));

    
        let coils = ctx
            .read_coils(0, NUM_BINS)
            .await
            .map_err(|e| Status::internal(format!("Modbus error: {}", e)))?;

        // check if any coil is High - > occupied        
        let accepted = coils.into_iter().any(|bit| bit);

        println!(
            "[Package Acceptance] robot={} hub={} -> Package Acceptance={}",
            req.robot_id, req.del_hub_id, accepted);

        // your acceptance logic here; for now always “true”
        Ok(Response::new(rpc::PackageAcceptanceResp { accepted }))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // try to connect to Modbus, retrying until successful
    let socket_addr = "127.0.0.1:5020".parse().unwrap();
    let ctx: Context = loop {
        match tcp::connect(socket_addr).await {
            Ok(ctx) => {
                println!("Modbus connected!");
                break ctx;
            }
            Err(e) => {
                eprintln!("Modbus connect failed, retrying in 1s: {:?}", e);
                sleep(Duration::from_secs(1)).await;
            }
        }
    };

    let shared = Arc::new(Mutex::new(ctx));
    let svc = SharedModbus { client: shared.clone() };

    let addr = "[::1]:50051".parse()?;
    println!("gRPC server listening on {}", addr);

    Server::builder()
        // add both of your services
        .add_service(
            rpc::delivery_hub_status_server::DeliveryHubStatusServer::new(svc.clone())
        )
        .add_service(
            rpc::package_status_server::PackageStatusServer::new(svc)
        )
        .serve(addr)
        .await?;

    Ok(())
}
