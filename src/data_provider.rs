pub trait DataProvider<T> {
    fn get_chunk(&self) -> T;
    fn set_chunk(&self, chunk: T);
}

pub struct SerialDataProvider {}

impl<T> DataProvider<T> for SerialDataProvider {
    fn get_chunk(&self) -> T {
        todo!()
    }

    fn set_chunk(&self, chunk: T) {
        todo!()
    }
}
