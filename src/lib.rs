// Copyright (c) 2018 Rouven Spreckels <n3vu0r@qu1x.org>
//
// Usage of the works is permitted provided that
// this instrument is retained with the works, so that
// any entity that uses the works is notified of this instrument.
//
// DISCLAIMER: THE WORKS ARE WITHOUT WARRANTY.

//! I²C API for Tokio

// TODO #![deny(missing_docs)]

extern crate tokio;
extern crate tokio_threadpool;
extern crate i2c_linux_sys;
#[macro_use] extern crate bitflags;
#[macro_use] extern crate smallvec;

use tokio::prelude::{*, Async::*};
use i2c_linux_sys::*;
use smallvec::{Array, SmallVec};

use std::mem::uninitialized;
use std::os::unix::io::{AsRawFd, IntoRawFd, RawFd};
use std::io::{self, Read, ErrorKind::*};
use std::path::Path;
use std::fs::{File, OpenOptions};
use std::time::Duration;

pub mod prelude {
	pub use tokio::prelude::*;
	pub use super::{
		Master,
		Functionality,
		Message,
		MessageFlags,
		TEN_BIT,
	};
	pub use smallvec::SmallVec;
	pub use std::time::Duration;
}

pub struct Message<A> where
	A: Array<Item = u8> + Send + 'static
{
	pub address: u16,
	pub data: SmallVec<A>,
	pub flags: MessageFlags,
}

impl<A> Message<A> where
	A: Array<Item = u8> + Send + 'static
{
	pub fn is_read(&self) -> bool {
		self.flags.contains(MessageFlags::READ)
	}
	pub fn is_write(&self) -> bool {
		!self.is_read()
	}
	fn as_i2c_msg(&mut self) -> i2c_msg {
		let mut flags = Flags::from_bits_truncate(self.flags.bits());
		if is_ten_bit(self.address)
			{ flags.insert(Flags::TEN) };
		i2c_msg {
			addr: to_address(self.address),
			flags,
			len: self.data.len() as u16,
			buf: self.data.as_mut_ptr(),
		}
	}
}

bitflags! {
	pub struct Functionality: u32 {
		const I2C = I2C_FUNC_I2C;
		const TEN_BIT_ADDR = I2C_FUNC_10BIT_ADDR;
		const PROTOCOL_MANGLING = I2C_FUNC_PROTOCOL_MANGLING;
		const SMBUS_PEC = I2C_FUNC_SMBUS_PEC;
		const NO_START = I2C_FUNC_NOSTART;
		const SLAVE = I2C_FUNC_SLAVE;
		const SMBUS_BLOCK_PROC_CALL = I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
		const SMBUS_QUICK = I2C_FUNC_SMBUS_QUICK;
		const SMBUS_READ_BYTE = I2C_FUNC_SMBUS_READ_BYTE;
		const SMBUS_WRITE_BYTE = I2C_FUNC_SMBUS_WRITE_BYTE;
		const SMBUS_READ_BYTE_DATA = I2C_FUNC_SMBUS_READ_BYTE_DATA;
		const SMBUS_WRITE_BYTE_DATA = I2C_FUNC_SMBUS_WRITE_BYTE_DATA;
		const SMBUS_READ_WORD_DATA = I2C_FUNC_SMBUS_READ_WORD_DATA;
		const SMBUS_WRITE_WORD_DATA = I2C_FUNC_SMBUS_WRITE_WORD_DATA;
		const SMBUS_PROC_CALL = I2C_FUNC_SMBUS_PROC_CALL;
		const SMBUS_READ_BLOCK_DATA = I2C_FUNC_SMBUS_READ_BLOCK_DATA;
		const SMBUS_WRITE_BLOCK_DATA = I2C_FUNC_SMBUS_WRITE_BLOCK_DATA;
		const SMBUS_READ_I2C_BLOCK = I2C_FUNC_SMBUS_READ_I2C_BLOCK;
		const SMBUS_WRITE_I2C_BLOCK = I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;
		const SMBUS_HOST_NOTIFY = I2C_FUNC_SMBUS_HOST_NOTIFY;
		const SMBUS_BYTE = I2C_FUNC_SMBUS_BYTE;
		const SMBUS_BYTE_DATA = I2C_FUNC_SMBUS_BYTE_DATA;
		const SMBUS_WORD_DATA = I2C_FUNC_SMBUS_WORD_DATA;
		const SMBUS_BLOCK_DATA = I2C_FUNC_SMBUS_BLOCK_DATA;
		const SMBUS_I2C_BLOCK = I2C_FUNC_SMBUS_I2C_BLOCK;
		const SMBUS_EMUL = I2C_FUNC_SMBUS_EMUL;
	}
}

bitflags! {
	#[derive(Default)]
	pub struct MessageFlags: u16 {
		const READ = I2C_M_RD;
		const EXACT = EXACT;
		const RECEIVE_LEN = I2C_M_RECV_LEN;
		const NACK = I2C_M_NO_RD_ACK;
		const IGNORE_NACK = I2C_M_IGNORE_NAK;
		const REVERSE_RW = I2C_M_REV_DIR_ADDR;
		const NO_START = I2C_M_NOSTART;
		const STOP = I2C_M_STOP;
	}
}

const EXACT: u16 = 0x0100;

impl From<Functionality> for MessageFlags {
	fn from(functionality: Functionality) -> MessageFlags {
		let mut flags = MessageFlags::READ | MessageFlags::EXACT
			| MessageFlags::RECEIVE_LEN;
		if functionality.contains(Functionality::PROTOCOL_MANGLING) {
			flags.insert(MessageFlags::NACK);
			flags.insert(MessageFlags::IGNORE_NACK);
			flags.insert(MessageFlags::REVERSE_RW);
			flags.insert(MessageFlags::STOP);
		}
		if functionality.contains(Functionality::NO_START) {
			flags.insert(MessageFlags::NO_START);
		}
		flags
	}
}

pub const TEN_BIT: u16 = 0xa000;

fn is_ten_bit(address: u16) -> bool {
	address & TEN_BIT == TEN_BIT
}

fn to_address(address: u16) -> u16 {
	address & !TEN_BIT
}

pub struct Master<F> where
	F: AsRawFd + Send + 'static,
{
	bus: F,
	address: Option<u16>,
	functionality: Functionality,
}

impl<F> Master<F> where
	F: AsRawFd + Send + 'static,
{
	pub fn into_inner(self) -> F {
		self.bus
	}
	pub fn inner_ref(&self) -> &F {
		&self.bus
	}
	pub fn inner_mut(&mut self) -> &mut F {
		&mut self.bus
	}
}

impl<F> AsRawFd for Master<F> where
	F: AsRawFd + Send + 'static,
{
	fn as_raw_fd(&self) -> RawFd {
		self.bus.as_raw_fd()
	}
}

impl<F> IntoRawFd for Master<F> where
	F: AsRawFd + IntoRawFd + Send + 'static,
{
	fn into_raw_fd(self) -> RawFd {
		self.bus.into_raw_fd()
	}
}

impl Master<File> {
	pub fn from_path<P>(path: P)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static where
		P: AsRef<Path> + Send + 'static,
	{
		FromPathFuture { path: Some(path) }
	}
}

impl<F> Master<F> where
	F: AsRawFd + Send + 'static,
{
	pub fn set_retries(self, retries: usize)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SetRetriesFuture { item: Some(self), retries }
	}
	pub fn set_timeout(self, duration: Duration)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SetTimeoutFuture { item: Some(self), duration }
	}
	pub fn set_slave_address(self, address: u16)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SetSlaveAddressFuture { item: Some(self), address }
	}
	pub fn functionality(&self) -> Functionality {
		self.functionality
	}
	pub fn transfer_flags(&self) -> MessageFlags {
		self.functionality.into()
	}
	pub fn transfer<A, M>(self, messages: SmallVec<M>)
	-> impl Future<Item = (SmallVec<M>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
		M: Array<Item = Message<A>> + Send + 'static,
	{
		assert!(messages.len() <= I2C_RDWR_IOCTL_MAX_MSGS);
		TransferFuture { item: Some((messages, self)) }
	}
	pub fn transfers<A, M, T>(self, transfers: SmallVec<T>)
	-> impl Future<Item = (SmallVec<T>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
		M: Array<Item = Message<A>> + Send + 'static,
		T: Array<Item = SmallVec<M>> + Send + 'static,
	{
		transfers.iter().for_each(|messages|
			assert!(messages.len() <= I2C_RDWR_IOCTL_MAX_MSGS));
		TransfersFuture { item: Some((transfers, self)) }
	}
	pub fn read_block_data<A>(self, command: u8,
		data: SmallVec<A>, exact: bool)
	-> impl Future<Item = (SmallVec<A>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		ReadBlockDataFuture { item: Some((data, self)), command, exact }
	}
	pub fn write_block_data<A>(self, command: u8, data: SmallVec<A>)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		WriteBlockDataFuture { item: Some((data, self)), command }
	}
}

impl<F> Master<F> where
	F: AsRawFd + Read + Send + 'static,
{
	pub fn read<A>(self, data: SmallVec<A>, exact: bool)
	-> impl Future<Item = (SmallVec<A>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		ReadFuture { item: Some((data, self)), exact }
	}
}

impl<F> Master<F> where
	F: AsRawFd + Write + Send + 'static,
{
	pub fn write<A>(self, data: SmallVec<A>, exact: bool)
	-> impl Future<Item = (SmallVec<A>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		WriteFuture { item: Some((data, self)), exact }
	}
}

impl<F> Master<F> where
	F: AsRawFd + Send + 'static,
{
	pub fn smbus_set_pec(self, pec: bool)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SmbusSetPecFuture { item: Some(self), pec }
	}
	pub fn smbus_write_quick(self, read_bit: bool)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SmbusWriteQuickFuture { item: Some(self), read_bit }
	}
	pub fn smbus_read_byte(self)
	-> impl Future<Item = (u8, Self), Error = io::Error> + Send + 'static {
		SmbusReadByteFuture { item: Some(self) }
	}
	pub fn smbus_write_byte(self, byte: u8)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SmbusWriteByteFuture { item: Some(self), byte }
	}
	pub fn smbus_read_byte_data<A>(self, command: u8)
	-> impl Future<Item = (u8, Self), Error = io::Error> + Send + 'static {
		SmbusReadByteDataFuture { item: Some(self), command }
	}
	pub fn smbus_write_byte_data(self, command: u8, byte: u8)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SmbusWriteByteDataFuture { item: Some(self), command, byte }
	}
	pub fn smbus_read_word_data(self, command: u8)
	-> impl Future<Item = (u16, Self), Error = io::Error> + Send + 'static {
		SmbusReadWordDataFuture { item: Some(self), command }
	}
	pub fn smbus_write_word_data(self, command: u8, word: u16)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static {
		SmbusWriteWordDataFuture { item: Some(self), command, word }
	}
	pub fn smbus_process_call(self, command: u8, word: u16)
	-> impl Future<Item = (u16, Self), Error = io::Error> + Send + 'static {
		SmbusProcessCallFuture { item: Some(self), command, word }
	}
	pub fn smbus_read_block_data<A>(self, command: u8,
		data: SmallVec<A>, exact: bool)
	-> impl Future<Item = (SmallVec<A>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		SmbusReadBlockDataFuture { item: Some((data, self)), command, exact }
	}
	pub fn smbus_write_block_data<A>(self, command: u8, data: SmallVec<A>)
	-> impl Future<Item = Self, Error = io::Error> + Send + 'static where
		A: Array<Item = u8> + Send + 'static,
	{
		SmbusWriteBlockDataFuture { item: Some((data, self)), command }
	}
	pub fn smbus_block_process_call<A, B>(self, command: u8,
		write_data: SmallVec<A>, read_data: SmallVec<B>, exact: bool)
	-> impl Future<Item = (SmallVec<B>, Self), Error = io::Error>
	+ Send + 'static where
		A: Array<Item = u8> + Send + 'static,
		B: Array<Item = u8> + Send + 'static,
	{
		SmbusBlockProcessCallFuture {
			item: Some((write_data, read_data, self)),
			command,
			exact,
		}
	}
}

struct FromPathFuture<P> where
	P: AsRef<Path> + Send + 'static,
{
	path: Option<P>,
}

impl<P> Future for FromPathFuture<P> where
	P: AsRef<Path> + Send + 'static,
{
	type Item = Master<File>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let path = self.path.take().expect(ERR_RESOLVED);
		if let Ready(master) = blocking_io(|| {
			let bus = OpenOptions::new().read(true).write(true).open(&path)?;
			let bits = i2c_get_functionality(bus.as_raw_fd())?.bits();
			Ok(Master { bus, address: None,
				functionality: Functionality::from_bits_truncate(bits) })
		})? {
			Ok(Ready(master))
		} else {
			self.path = Some(path);
			Ok(NotReady)
		}
	}
}

struct SetRetriesFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	retries: usize,
}

impl<F> Future for SetRetriesFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_set_retries(master.as_raw_fd(), self.retries)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SetTimeoutFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	duration: Duration,
}

impl<F> Future for SetTimeoutFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			let duration = self.duration.as_secs() as usize * 1_000
				+ self.duration.subsec_millis() as usize;
			i2c_set_timeout_ms(master.as_raw_fd(), duration)
		})? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SetSlaveAddressFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	address: u16,
}

impl<F> Future for SetSlaveAddressFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			let address = to_address(self.address);
			let ten_bit = is_ten_bit(self.address);
			if master.address.map_or(false, is_ten_bit) != ten_bit {
				i2c_set_slave_address_10bit(master.as_raw_fd(), ten_bit)?;
			}
			if master.address.map(to_address) != Some(address) {
				i2c_set_slave_address(master.as_raw_fd(), address, false)?;
			}
			Ok(())
		})? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct TransferFuture<F, A, M> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	M: Array<Item = Message<A>>,
{
	item: Option<(SmallVec<M>, Master<F>)>,
}

impl<F, A, M> Future for TransferFuture<F, A, M> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	M: Array<Item = Message<A>>,
{
	type Item = (SmallVec<M>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut messages, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_transfer(master.as_raw_fd(), &mut messages)
		)? {
			Ok(Ready((messages, master)))
		} else {
			self.item = Some((messages, master));
			Ok(NotReady)
		}
	}
}

struct TransfersFuture<F, A, M, T> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	M: Array<Item = Message<A>>,
	T: Array<Item = SmallVec<M>>,
{
	item: Option<(SmallVec<T>, Master<F>)>,
}

impl<F, A, M, T> Future for TransfersFuture<F, A, M, T> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	M: Array<Item = Message<A>>,
	T: Array<Item = SmallVec<M>>,
{
	type Item = (SmallVec<T>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut transfers, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			transfers.iter_mut().map(|mut messages|
				i2c_transfer(master.as_raw_fd(), &mut messages)).collect()
		)? {
			Ok(Ready((transfers, master)))
		} else {
			self.item = Some((transfers, master));
			Ok(NotReady)
		}
	}
}

struct ReadBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	command: u8,
	exact: bool,
}

impl<F, A> Future for ReadBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = (SmallVec<A>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut data, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			let functionality = master.functionality;
			if functionality.contains(Functionality::I2C)
			&& (!functionality.contains(Functionality::SMBUS_READ_I2C_BLOCK)
				|| data.len() > I2C_SMBUS_BLOCK_MAX
			) {
				if let Some(address) = master.address {
					let mut messages = SmallVec::<[Message<A>; 2]>::from_buf([
						Message {
							address: address,
							data: smallvec![self.command],
							flags: MessageFlags::empty(),
						},
						Message {
							address: address,
							data: data.clone(), // TODO
							flags: MessageFlags::READ,
						},
					]);
					i2c_transfer(master.as_raw_fd(), &mut messages)?;
					data = messages.swap_remove(1).data;
					Ok(())
				} else {
					Err(io::Error::new(io::ErrorKind::Other,
						"No slave address set"))
				}
			} else {
				data.truncate(I2C_SMBUS_BLOCK_MAX);
				i2c_smbus_read_block_data(master.as_raw_fd(),
				self.command, &mut data).and_then(|len|
					if self.exact && len != data.len() {
						Err(UnexpectedEof.into())
					} else {
						Ok(data.truncate(len))
					})
			}
		})? {
			Ok(Ready((data, master)))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

struct WriteBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	command: u8,
}

impl<F, A> Future for WriteBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (data, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			let functionality = master.functionality;
			if functionality.contains(Functionality::I2C)
			&& (!functionality.contains(Functionality::SMBUS_READ_I2C_BLOCK)
				|| data.len() > I2C_SMBUS_BLOCK_MAX
			) {
				if let Some(address) = master.address {
					if functionality.contains(Functionality::NO_START) {
						type Messages<A> = SmallVec<[Message<A>; 2]>;
						let mut messages = Messages::from_buf([
							Message {
								address: address,
								data: smallvec![self.command],
								flags: MessageFlags::empty(),
							},
							Message {
								address: address,
								data: data.clone(), // TODO
								flags: MessageFlags::NO_START,
							},
						]);
						i2c_transfer(master.as_raw_fd(), &mut messages)?;
					} else {
						let mut command_data = SmallVec::<A>::new();
						command_data.push(self.command);
						command_data.copy_from_slice(&data);
						type Messages<A> = SmallVec<[Message<A>; 1]>;
						let mut messages = Messages::from_buf([
							Message {
								address: address,
								data: command_data,
								flags: MessageFlags::empty(),
							},
						]);
						i2c_transfer(master.as_raw_fd(), &mut messages)?;
					}
					Ok(())
				} else {
					Err(io::Error::new(io::ErrorKind::InvalidInput,
						"No slave address set"))
				}
			} else {
				i2c_smbus_write_block_data(master.as_raw_fd(),
					self.command, &data)
			}
		})? {
			Ok(Ready(master))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

fn i2c_transfer<A, M>(fd: RawFd, messages: &mut SmallVec<M>)
-> io::Result<()> where
	A: Array<Item = u8> + Send + 'static,
	M: Array<Item = Message<A>>,
{
	let mut msgs = unsafe
		{ uninitialized::<[i2c_msg; I2C_RDWR_IOCTL_MAX_MSGS]>() };
	msgs.iter_mut().zip(messages.iter_mut())
		.for_each(|(msg, message)| *msg = message.as_i2c_msg());
	unsafe { i2c_rdwr(fd, &mut msgs[..messages.len()])? };
	msgs.iter().zip(messages.iter_mut()).map(|(msg, message)|
		if message.flags.contains(MessageFlags::READ | MessageFlags::EXACT)
		&& msg.len as usize != message.data.len() {
			Err(UnexpectedEof.into())
		} else {
			Ok(message.data.truncate(msg.len as usize))
		}
	).collect()
}

struct ReadFuture<F, A> where
	F: AsRawFd + Read + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	exact: bool,
}

impl<F, A> Future for ReadFuture<F, A> where
	F: AsRawFd + Read + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = (SmallVec<A>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut data, mut master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			master.bus.read(&mut data).and_then(|len|
				if self.exact && len != data.len() {
					Err(UnexpectedEof.into())
				} else {
					Ok(data.truncate(len))
				})
		)? {
			Ok(Ready((data, master)))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

struct WriteFuture<F, A> where
	F: AsRawFd + Write + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	exact: bool,
}

impl<F, A> Future for WriteFuture<F, A> where
	F: AsRawFd + Write + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = (SmallVec<A>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut data, mut master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			master.bus.write(&mut data.as_mut_slice()).and_then(|len|
				if self.exact && len != data.len() {
					Err(UnexpectedEof.into())
				} else {
					Ok(data.drain().take(len).for_each(drop))
				}
			)
		)? {
			Ok(Ready((data, master)))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

struct SmbusSetPecFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	pec: bool,
}

impl<F> Future for SmbusSetPecFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_pec(master.as_raw_fd(), self.pec)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusWriteQuickFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	read_bit: bool,
}

impl<F> Future for SmbusWriteQuickFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_smbus_write_quick(master.as_raw_fd(), match self.read_bit {
				true => i2c_linux_sys::SmbusReadWrite::Read,
				false => i2c_linux_sys::SmbusReadWrite::Write,
			})
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusReadByteFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
}

impl<F> Future for SmbusReadByteFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = (u8, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(byte) = blocking_io(||
			i2c_smbus_read_byte(master.as_raw_fd())
		)? {
			Ok(Ready((byte, master)))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusWriteByteFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	byte: u8,
}

impl<F> Future for SmbusWriteByteFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_smbus_write_byte(master.as_raw_fd(), self.byte)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusReadByteDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	command: u8,
}

impl<F> Future for SmbusReadByteDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = (u8, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(byte) = blocking_io(||
			i2c_smbus_read_byte_data(master.as_raw_fd(), self.command)
		)? {
			Ok(Ready((byte, master)))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusWriteByteDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	command: u8,
	byte: u8,
}

impl<F> Future for SmbusWriteByteDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_smbus_write_byte_data(master.as_raw_fd(),
				self.command, self.byte)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusReadWordDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	command: u8,
}

impl<F> Future for SmbusReadWordDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = (u16, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(word) = blocking_io(||
			i2c_smbus_read_word_data(master.as_raw_fd(), self.command)
		)? {
			Ok(Ready((word, master)))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusWriteWordDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	command: u8,
	word: u16,
}

impl<F> Future for SmbusWriteWordDataFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_smbus_write_word_data(master.as_raw_fd(),
				self.command, self.word)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusProcessCallFuture<F> where
	F: AsRawFd + Send + 'static,
{
	item: Option<Master<F>>,
	command: u8,
	word: u16,
}

impl<F> Future for SmbusProcessCallFuture<F> where
	F: AsRawFd + Send + 'static,
{
	type Item = (u16, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let master = self.item.take().expect(ERR_RESOLVED);
		if let Ready(word) = blocking_io(||
			i2c_smbus_process_call(master.as_raw_fd(), self.command, self.word)
		)? {
			Ok(Ready((word, master)))
		} else {
			self.item = Some(master);
			Ok(NotReady)
		}
	}
}

struct SmbusReadBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	command: u8,
	exact: bool,
}

impl<F, A> Future for SmbusReadBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = (SmallVec<A>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (mut data, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			data.truncate(I2C_SMBUS_BLOCK_MAX);
			i2c_smbus_read_block_data(master.as_raw_fd(),
				self.command, &mut data
			).and_then(|len| if self.exact && len != data.len() {
				Err(UnexpectedEof.into())
			} else {
				Ok(data.truncate(len))
			})
		})? {
			Ok(Ready((data, master)))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

struct SmbusWriteBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, Master<F>)>,
	command: u8,
}

impl<F, A> Future for SmbusWriteBlockDataFuture<F, A> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
{
	type Item = Master<F>;
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (data, master) = self.item.take().expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(||
			i2c_smbus_write_block_data(master.as_raw_fd(), self.command, &data)
		)? {
			Ok(Ready(master))
		} else {
			self.item = Some((data, master));
			Ok(NotReady)
		}
	}
}

struct SmbusBlockProcessCallFuture<F, A, B> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	B: Array<Item = u8> + Send + 'static,
{
	item: Option<(SmallVec<A>, SmallVec<B>, Master<F>)>,
	command: u8,
	exact: bool,
}

impl<F, A, B> Future for SmbusBlockProcessCallFuture<F, A, B> where
	F: AsRawFd + Send + 'static,
	A: Array<Item = u8> + Send + 'static,
	B: Array<Item = u8> + Send + 'static,
{
	type Item = (SmallVec<B>, Master<F>);
	type Error = io::Error;

	fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
		let (write_data, mut read_data, master) = self.item.take()
			.expect(ERR_RESOLVED);
		if let Ready(()) = blocking_io(|| {
			read_data.truncate(I2C_SMBUS_BLOCK_MAX);
			i2c_smbus_block_process_call(master.as_raw_fd(), self.command,
				&write_data, &mut read_data).and_then(|len|
					if self.exact && len != read_data.len() {
						Err(UnexpectedEof.into())
					} else {
						Ok(read_data.truncate(len))
					}
				)
		})? {
			Ok(Ready((read_data, master)))
		} else {
			self.item = Some((write_data, read_data, master));
			Ok(NotReady)
		}
	}
}

fn blocking_io<F, T>(f: F) -> Poll<T, io::Error> where
	F: FnOnce() -> io::Result<T>
{
	match tokio_threadpool::blocking(f) {
		Ok(Ready(Ok(ok))) => Ok(ok.into()),
		Ok(Ready(Err(err))) => match err.kind() {
			WouldBlock => Ok(NotReady),
			_ => Err(err),
		},
		Ok(NotReady) => Ok(NotReady),
		Err(err) => Err(io::Error::new(Other, err)),
	}
}

const ERR_RESOLVED: &str = "I²C Future already resolved";
