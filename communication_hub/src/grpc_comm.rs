use tonic::{transport::Server, Request, Response, Status};

use rpc::package_status_server::{PackageStatus, PackageStatusServer};
use rpc::{PackageAcceptanceReq, PackageAcceptanceResp};

pub mod rpc {
    tonic::include_proto!("robot_to_del");
}

#[derive(Debug, Default)]
pub struct MyPackageStatusService;

#[tonic::async_trait]
impl PackageStatus for MyPackageStatusService {
    async fn robot_to_server(
        &self,
        request: Request<PackageAcceptanceReq>,
    ) -> Result<Response<PackageAcceptanceResp>, Status> {
        let req = request.into_inner();
        println!(
            "Robot {} requests drop at hub {} slot {:?}",
            req.robot_id,
            req.del_hub_id,
            // slot_id is a Vec<u8>, so we can convert to String or hex as needed:
            std::str::from_utf8(&req.slot_id).ok()
        );

        // TODO: your acceptance logic here
        let accepted = true;

        Ok(Response::new(PackageAcceptanceResp { accepted }))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "[::1]:50051".parse()?;
    let svc = MyPackageStatusService::default();

    println!("PackageStatusServer listening on {}", addr);
    Server::builder()
        .add_service(PackageStatusServer::new(svc))
        .serve(addr)
        .await?;

    Ok(())
}
