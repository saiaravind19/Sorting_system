use rpc::package_status_client::PackageStatusClient;
use rpc::PackageAcceptanceReq;

pub mod rpc {
    tonic::include_proto!("robot_to_del"); // Same proto package as the server
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to the gRPC server
    let mut client = PackageStatusClient::connect("http://[::1]:50051").await?;

    // Build request payload
    let request = tonic::Request::new(PackageAcceptanceReq {
        robot_id: "R2D2".into(),
        del_hub_id: "Hub-Alpha".into(),
        slot_id: b"Slot-42".to_vec(), // Bytes field
    });

    // Send RPC
    let response = client.robot_to_server(request).await?;

    // Print result
    println!("Response from server: accepted = {}", response.into_inner().accepted);

    Ok(())
}
