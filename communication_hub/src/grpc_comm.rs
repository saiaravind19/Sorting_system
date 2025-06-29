use tonic::{transport::Server, Request, Response, Status};


use rpc::greeter_server::{Greeter, GreeterServer};
use rpc::{HelloReply, HelloRequest};


pub mod rpc {
    tonic::include_proto!("helloworld");
}

#[derive(Debug, Default)]
pub struct MyGreeter {}

#[tonic::async_trait]
impl Greeter for MyGreeter {
    async fn say_hello(
        &self,
        request: Request<HelloRequest>,
    ) -> Result<Response<HelloReply>, Status> {
        println!("Got a request: {:?}", request);

        let reply = HelloReply {
            message: format!("Hello {}!", request.into_inner().name),
        };

        Ok(Response::new(reply))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting gRPC server...");
    let addr = "[::1]:50051".parse()?;
    let greeter = MyGreeter::default();
    println!("Greeter service listening on {}", addr);
    Server::builder()
        .add_service(GreeterServer::new(greeter))
        .serve(addr)
        .await?;

    Ok(())
}