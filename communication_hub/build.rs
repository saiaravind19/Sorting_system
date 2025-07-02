fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Use tonic_build::configure().compile() to compile multiple .proto files at once
    tonic_build::configure()
        .compile(
            &[
                "proto/del_hub_availability.proto",
                "proto/del_hub_pack_accept.proto",
            ],
            &[
                "proto", // include path(s)
            ],
        )?;
    Ok(())
}
