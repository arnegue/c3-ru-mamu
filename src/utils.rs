// Simple u8-buffer to string converter
pub fn buffer_to_string(buffer: &[u8], size: usize) -> String {
    let mut result = String::new();
    for i in 0..size {
        result.push(buffer[i] as char);
    }
    result
}
