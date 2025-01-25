use thiserror::Error;
use xum1541::Error;

#[derive(Debug, Error)]
pub enum AppError {
    #[error("xum1541 error: {error}")]
    Error { error: Error },

    #[error("App error: {message}")]
    App { message: String },
}

impl From<Error> for AppError {
    fn from(error: Error) -> Self {
        AppError::Error { error }
    }
}

impl From<String> for AppError {
    fn from(message: String) -> Self {
        AppError::App { message }
    }
}
