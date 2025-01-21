use thiserror::Error;
use xum1541::Xum1541Error;

#[derive(Debug, Error)]
pub enum AppError {
    #[error("xum1541 error: {error}")]
    Xum1541Error{ error: Xum1541Error},
    
    #[error("App error: {message}")]
    App{message: String},
}

impl From<Xum1541Error> for AppError {
    fn from(error: Xum1541Error) -> Self {
        AppError::Xum1541Error { error }
    }
}

impl From<String> for AppError {
    fn from(message: String) -> Self {
        AppError::App { message }
    }
}