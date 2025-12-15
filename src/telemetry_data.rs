type TelemetryDataType = Option<f32>; // TODO make a time-based option

#[derive(Clone, Copy)]
pub struct TelemetryData {
    pub speed: TelemetryDataType,
    pub depth: TelemetryDataType,
    pub course: TelemetryDataType, // add more fields as needed
}
