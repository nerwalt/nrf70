use core::cell::{Cell, RefCell};
use core::future::{Future, poll_fn};
use core::ptr;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::waitqueue::WakerRegistration;

use futures::Stream;

use crate::Error;
use crate::bindings::nrf_wifi_host_rpu_msg_type;


#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Item {
    UmacInfo,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Action {
    Boot(*const [u8]),
    Command((nrf_wifi_host_rpu_msg_type, bool, *const [u8], Option<*mut [u8]>)),
    Get((Item, *mut [u8])),
    WaitForDone,
}

#[derive(Clone, Copy)]
enum ActionStateInner {
    Pending(Action),
    Sent { response_buffer: Option<*mut [u8]> },
    Done {
        result: Result<Option<usize>, Error>,
    },
}

struct Wakers {
    control: WakerRegistration,
    runner: WakerRegistration,
    stream: WakerRegistration,
}

impl Wakers {
    const fn new() -> Self {
        Self {
            control: WakerRegistration::new(),
            runner: WakerRegistration::new(),
            stream: WakerRegistration::new(),
        }
    }
}

pub type StreamResponse = heapless::Vec<u8, 1600>;
pub const STREAM_CAP: usize = 4;
pub type StreamResponseChannel = Channel<CriticalSectionRawMutex, StreamResponse, STREAM_CAP>;

static STREAM_RESONSE_CHANNEL: StreamResponseChannel = StreamResponseChannel::new();

pub struct ResponseStream<'a> {
    receiver: Receiver<'static, CriticalSectionRawMutex, StreamResponse, STREAM_CAP>,
    state: &'a ActionState,
    done: bool,
}

impl<'a> Stream for ResponseStream<'a> {
    type Item = Result<StreamResponse, Error>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();

        if this.done {
            return Poll::Ready(None);
        }

        match this.receiver.poll_receive(cx) {
            Poll::Ready(chunk) => return Poll::Ready(Some(Ok(chunk))),
            Poll::Pending => {}
        }

        if let ActionStateInner::Done { result } = this.state.state.get() {
            this.done = true;
            return match result {
                Ok(_size) => Poll::Ready(None),
                Err(e) => Poll::Ready(Some(Err(e))),
            };
        }

        this.state.register_stream(cx.waker());
        Poll::Pending
    }
}

impl<'a> core::fmt::Debug for ResponseStream<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ResponseStream").finish()
    }
}

pub struct ActionState {
    state: Cell<ActionStateInner>,
    wakers: RefCell<Wakers>,
    chan: &'static StreamResponseChannel,
}

#[allow(dead_code)]
impl ActionState {
    pub const fn new() -> Self {
        Self {
            state: Cell::new(ActionStateInner::Done { result: Ok(None) }),
            wakers: RefCell::new(Wakers::new()),
            chan: &STREAM_RESONSE_CHANNEL,
        }
    }

    fn wake_control(&self) { self.wakers.borrow_mut().control.wake(); }
    fn register_control(&self, w: &Waker) { self.wakers.borrow_mut().control.register(w); }
    fn wake_runner(&self) { self.wakers.borrow_mut().runner.wake(); }
    fn register_runner(&self, w: &Waker) { self.wakers.borrow_mut().runner.register(w); }
    fn wake_stream(&self) { self.wakers.borrow_mut().stream.wake(); }
    fn register_stream(&self, w: &Waker) { self.wakers.borrow_mut().stream.register(w); }

    pub fn wait_complete(&self) -> impl Future<Output = Result<Option<usize>, Error>> + '_ {
        poll_fn(|cx| {
            if let ActionStateInner::Done { result } = self.state.get() {
                Poll::Ready(result)
            } else {
                self.register_control(cx.waker());
                Poll::Pending
            }
        })
    }

    pub fn wait_pending(&self) -> impl Future<Output = Action> + '_ {
        poll_fn(|cx| {
            if let ActionStateInner::Pending(pending) = self.state.get() {
                self.state.set(ActionStateInner::Sent {
                    response_buffer: match pending {
                        Action::Command((_, _, _, response_buffer)) => response_buffer,
                        Action::Get((_, response_buffer)) => Some(response_buffer),
                        _ => None,
                    },
                });

                Poll::Ready(pending)
            } else {
                self.register_runner(cx.waker());
                Poll::Pending
            }
        })
    }

    pub fn cancel(&self) {
        self.state.set(ActionStateInner::Done { result: Ok(None) });
    }

    pub async fn issue(&self, action: Action) -> Result<Option<usize>, Error> {
        match self.state.get() {
            ActionStateInner::Done { result: _ } => (),
            _ => return Err(Error::Busy),
        }

        self.state.set(ActionStateInner::Pending(action));

        self.wake_runner();
        self.wait_complete().await
    }

    pub fn respond(&self, result: Result<Option<*const [u8]>, Error>) {
        if let ActionStateInner::Sent { response_buffer } = self.state.get() {
            // Response buffer may be a value (given by the optional) and should be filled under the following conditions:
            //
            // * The result is OK and its optional contains a value
            // * The response buffer has enough space for the result
            fn get_result(
                result: Result<Option<*const [u8]>, Error>,
                response_buffer: Option<*mut [u8]>,
            ) -> Result<Option<usize>, Error> {
                match result {
                    Ok(Some(result_data)) => unsafe {
                        match response_buffer {
                            Some(response_buffer_ptr) => {
                                let result_data_length = result_data.len();
                                let response_buffer: &mut [u8] = &mut *response_buffer_ptr;

                                if response_buffer.len() < result_data_length {
                                    return Err(Error::BufferTooSmall);
                                }

                                let result_data_ptr: &[u8] = &*result_data;

                                ptr::copy_nonoverlapping(
                                    result_data_ptr.as_ptr(),
                                    response_buffer.as_mut_ptr(),
                                    result_data_length,
                                );

                                Ok(Some(result_data_length))
                            }
                            None => Ok(None),
                        }
                    },
                    Ok(None) => Ok(None),
                    Err(e) => Err(e),
                }
            }

            self.state.set(ActionStateInner::Done {
                result: get_result(result, response_buffer),
            });

            self.wake_control();
        } else {
            warn!("Acking action, but no pending action");
        }
    }

    /// Issue an action that expects a stream of responses
    pub fn issue_stream(&self, action: Action) -> Result<ResponseStream<'_>, Error> {
        if !matches!(self.state.get(), ActionStateInner::Done { .. }) {
            return Err(Error::Busy);
        }

        self.chan.clear();

        let receiver = self.chan.receiver();
        self.state.set(ActionStateInner::Pending(action));
        self.wake_runner();
        Ok(ResponseStream { receiver, state: self, done: false })
    }


    /// Send a resopnse on the the response stream (valid only with `issue_stream`).
    pub fn respond_stream(&self, response: &[u8]) -> Result<(), Error> {
        match self.state.get() {
            ActionStateInner::Sent{..} => {
                let mut v = StreamResponse::new();
                if let Err(_) = v.extend_from_slice(response) {
                    return Err(Error::BufferTooSmall)
                }
                if let Err(_) = self.chan.try_send(v) {
                    return Err(Error::StreamTooSmall)
                }
            }
            _ => return Err(Error::InvalidState),
        }

        Ok(())
    }

    /// Finish a stream action
    pub fn finish_stream(&self, result: Result<Option<usize>, Error>) {
        self.state.set(ActionStateInner::Done { result });
        self.wake_control();
        self.wake_stream();
    }

}


