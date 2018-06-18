#!/usr/bin/env python3

import serial, struct, sys, time

class CanMsgEvent:
	def __init__(self, id, data):
		self.id = id
		self.data = data

	def __str__(self):
		return 'CanMsgEvent(%xh %s)' % (self.id, ' '.join(['%02X' % b for b in self.data]))

class ErrorFlagsEvent:
	def __init__(self, value):
		self.value = value

	def __str__(self):
		return 'ErrorFlagsEvent(%04X)' % self.value

class IfIdEvent:
	def __init__(self, if_id):
		self.if_id = if_id

	def __str__(self):
		return 'IfIdEvent(%s)' % self.if_id

class StatusFlagsEvent:
	def __init__(self, value):
		self.value = value

	def __str__(self):
		return 'StatusFlagsEvent(%08X)' % self.value

class Ocarina:
	def __init__(self, port):
		self.ser = serial.Serial(port, 115200, timeout=1)

		# flush RX buffers
		self.disable_message_forwarding()
		time.sleep(0.2)
		self.ser.flushInput()

	def close(self):
		self.ser.close()

	def read_event(self):
		type = self.ser.read(1)

		if type == b'\x01':
			id_ = b''

			while True:
				byte = self.ser.read(1)

				if byte == b'\n': break
				elif ord(byte) >= 0x20: id_ += byte

			return IfIdEvent(id_.decode())
		elif type == b'\x02':
			sid = struct.unpack('H', self.ser.read(2))[0]
			length = ord(self.ser.read(1))
			data = self.ser.read(length)

			return CanMsgEvent(sid, data)
		elif type == b'R':
			eid = struct.unpack('I', self.ser.read(4))[0]
			length = ord(self.ser.read(1))
			data = self.ser.read(length)

			return CanMsgEvent(eid, data)
		elif type == b'e':
			value = struct.unpack('H', self.ser.read(2))[0]
			return ErrorFlagsEvent(value)
		elif type == b's':
			value = struct.unpack('I', self.ser.read(4))[0]
			return StatusFlagsEvent(value)
		elif type == b'\xAA':
			return 'Ocarina Heartbeat'

	def enter_link_mode(self, dev_uid):
		self.ser.write(b'\x10')
		self.ser.write(struct.pack('B', dev_uid))

	def disable_message_forwarding(self):
		self.ser.write(b'\xF0')

	def enable_message_forwarding(self):
		self.ser.write(b'\xF1')

	def send_message(self, sid, data = b''):
		self.ser.write(b'\x02')
		self.ser.write(struct.pack('HB', sid, len(data)))
		self.ser.write(data)

	def send_message_ext(self, eid, data = b''):
		self.ser.write(b'T')
		self.ser.write(struct.pack('IB', eid, len(data)))
		self.ser.write(data)

	def test_filter(self):
		self.ser.write(b'\xF2\xFF\xFF\x66\x00')

	def query_error_flags(self):
		self.ser.write(b'e')

	def query_interface(self):
		self.ser.write(b'\x01')

	def query_status_flags(self):
		self.ser.write(b's')

	def wait_for_sid(self, sid):
		while True:
			ev = ocarina.read_event()
			#print(ev)

			if isinstance(ev, CanMsgEvent) and ev.sid == sid:
				return ev

class TXDL:
	def __init__(self, ocarina, sid):
		self.ocarina = ocarina
		self.sid = sid

	def detect(self):
		sid = self.sid | 0x00
		ocarina.send_message(sid, b'')
		ev = ocarina.wait_for_sid(sid)

		print('Downloader:', ev.data.decode())

		return ev.data == b'TXDLF103'

	def erase_page(self):
		sid = self.sid | 0x0E
		ocarina.send_message(sid)
		ocarina.wait_for_sid(sid)

	def read_dword(self):
		sid = self.sid | 0x02
		ocarina.send_message(sid)
		ev = ocarina.wait_for_sid(sid)
		return ev.data

	def reset(self):
		sid = self.sid | 0x0F
		ocarina.send_message(sid)

	def set_address(self, address):
		sid = self.sid | 0x01
		ocarina.send_message(sid, struct.pack('I', address))
		ocarina.wait_for_sid(sid)

	def write_dword(self, bytes):
		sid = self.sid | 0x03
		ocarina.send_message(sid, bytes)
		ocarina.wait_for_sid(sid)

if __name__ == "__main__":
	port = sys.argv[1]
	cmd = sys.argv[2]

	ocarina = Ocarina(port)

	args = sys.argv[3:]

	if False: pass
	elif cmd == 'detect':
		ocarina.enable_message_forwarding()

		devs = {}

		t_end = time.time() + 3

		while time.time() < t_end:
			ev = ocarina.read_event()

			if isinstance(ev, CanMsgEvent):
				if ev.sid >= 96 and ev.sid < 112:
					dev = ev.sid & 15
					devs[dev] = {
						'preambule': ev.data[0],
						'uid': ev.data[1],
						}

		for key in sorted(devs):
			print(key, devs[key])
	elif cmd == 'enter_link_mode':
		ocarina.enter_link_mode(int(args[0]))
	elif cmd == 'get_sdo':
		ocarina.enable_message_forwarding()
		node_id = int(args[0])
		object_index = 0x1200 #int(args[1], 0)

		# calculate msg id
		msg_id = 0x600 + node_id

		# send request
		CCS_UPLOAD = 0x40
		ocarina.send_message_ext(msg_id, struct.pack('BBBB', CCS_UPLOAD, object_index % 256, object_index // 256, 0))

		t_end = time.time() + 3

		while time.time() < t_end:
			ev = ocarina.read_event()

			if isinstance(ev, CanMsgEvent):
				if ev.id == msg_id:
					print(ev)
					data = ev.data[4:]
					print(data)
					break
	elif cmd == 'query':
		ocarina.query_interface()
		ocarina.query_status_flags()
		ocarina.query_error_flags()
		count = 0

		while count < 3:
			ev = ocarina.read_event()

			if isinstance(ev, IfIdEvent):
				print(ev.if_id)
				count += 1
			elif isinstance(ev, ErrorFlagsEvent) or isinstance(ev, StatusFlagsEvent):
				print(ev)
				count += 1
	elif cmd == 'send':
		sid = int(args[0], 0)
		data = bytearray.fromhex(args[1]) if len(args) > 1 else b''
		ocarina.send_message(sid, data)
	elif cmd == 'sniff':
		if len(args) >= 1:
			sid = int(args[0], 0)
		else:
			sid = None

		ocarina.enable_message_forwarding()

		while True:
			ev = ocarina.read_event()

			if sid != None and isinstance(ev, CanMsgEvent) and ev.sid != sid:
				continue

			print(ev)
	elif cmd == 'spam':
		while True:
			ocarina.send_message(0x094, b'\x11')
			time.sleep(1)
	elif cmd == 'testfilt':
		ocarina.test_filter()
	elif cmd == 'txdl-flash':
		txdl = TXDL(ocarina, int(args[0], 0))

		with open(args[1], 'rb') as f:
			image = f.read()

		start = int(args[2], 0)

		if txdl.detect():
			page_size = 1024
			num_pages = (len(image) + page_size - 1) // page_size
			print('Flashing %d pages\t' % num_pages, end='')

			txdl.set_address(start)

			for page in range(num_pages):
				page_data = image[page * page_size:(page + 1) * page_size]
				txdl.erase_page()

				for dw in range(len(page_data) // 8):
					offset = page * page_size + dw * 8
					txdl.write_dword(image[offset:offset + 8])

				print('.', end='')
				sys.stdout.flush()

			print(' \u2713')

			print('Verifying\t\t', end='')
			txdl.set_address(start)

			for page in range(num_pages):
				page_data = image[page * page_size:(page + 1) * page_size]

				for dw in range(len(page_data) // 8):
					offset = page * page_size + dw * 8
					readback = txdl.read_dword()

					if readback != image[offset:offset + 8]:
						print('VERIFY ERROR @ %08Xh: %s vs %s' % (start + offset, readback, image[offset:offset + 8]))

				print('.', end='')
				sys.stdout.flush()

			print(' \u2713')

			txdl.reset()
	elif cmd == 'txdl-id':
		txdl = TXDL(ocarina, int(args[0], 0))
		txdl.detect()
	else:
		abort()
