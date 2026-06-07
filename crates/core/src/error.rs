use thiserror::Error;

#[derive(Error, Debug)]
pub enum CoreError {
    #[error("kernel error: {0}")]
    Kernel(String),

    #[error("STEP parsing failed: {0}")]
    StepParse(String),

    #[error("invalid handle")]
    InvalidHandle,

    #[error("scale factor must be > 0")]
    InvalidScale,

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("serialization error: {0}")]
    Serde(#[from] serde_json::Error),

    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}
