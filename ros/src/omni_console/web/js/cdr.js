// @ts-check
/* Serialized ROS 2 messages (CDR), decoded in the browser. rosbridge's cbor-raw mode hands the page the bytes the
   publisher wrote, so the robot converts nothing: the map, the lidar scan and the MCU's samples come this way. The
   robot-side twin is omni_console/cdr.py. XCDR1: a 4-byte header (byte 1: 1 = little endian), then each field aligned
   to its size, counted from after the header. */

export class CdrReader {
  /** @param {Uint8Array} bytes */
  constructor(bytes) {
    this.u8 = bytes;
    this.dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.le = (bytes[1] & 1) === 1;
    this.p = 4;
  }
  /** @param {number} n */
  align(n) { this.p += (n - ((this.p - 4) % n)) % n; }
  uint8() { const v = this.dv.getUint8(this.p); this.p += 1; return v; }
  uint32() { this.align(4); const v = this.dv.getUint32(this.p, this.le); this.p += 4; return v; }
  int32() { this.align(4); const v = this.dv.getInt32(this.p, this.le); this.p += 4; return v; }
  float32() { this.align(4); const v = this.dv.getFloat32(this.p, this.le); this.p += 4; return v; }
  float64() { this.align(8); const v = this.dv.getFloat64(this.p, this.le); this.p += 8; return v; }
  string() {
    const n = this.uint32();
    const s = new TextDecoder().decode(this.u8.subarray(this.p, this.p + Math.max(0, n - 1)));
    this.p += n;
    return s;
  }
  stamp() { const sec = this.int32(); return sec + this.uint32() * 1e-9; }
  header() { const t = this.stamp(); return { t, frame: this.string() }; }
  /** A float32[] sequence, as a typed array (copied when the bytes aren't 4-aligned in memory). */
  float32Seq() {
    const n = this.uint32();
    this.align(4);
    const off = this.u8.byteOffset + this.p;
    let a;
    if (this.le && off % 4 === 0) a = new Float32Array(this.u8.buffer, off, n);
    else { a = new Float32Array(n); for (let i = 0; i < n; i++) a[i] = this.dv.getFloat32(this.p + 4 * i, this.le); }
    this.p += 4 * n;
    return a;
  }
  int8Seq() {
    const n = this.uint32();
    const a = new Int8Array(this.u8.buffer, this.u8.byteOffset + this.p, n);
    this.p += n;
    return a;
  }
}

/** rosbridge's cbor-raw message ({secs, nsecs, bytes}) -> the bytes. @param {any} m @returns {Uint8Array} */
export const rawBytes = (m) => (m.bytes instanceof Uint8Array ? m.bytes : new Uint8Array(m.bytes));

/** sensor_msgs/LaserScan. @param {Uint8Array} b */
export function decodeLaserScan(b) {
  const r = new CdrReader(b);
  const header = r.header();
  const [angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max] =
    [0, 0, 0, 0, 0, 0, 0].map(() => r.float32());
  const ranges = r.float32Seq();
  return { header, angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max, ranges };
}

/** nav_msgs/OccupancyGrid, in the shape the page used for rosbridge's JSON (info.width, info.origin.position.x ...).
 *  @param {Uint8Array} b */
export function decodeOccupancyGrid(b) {
  const r = new CdrReader(b);
  const header = r.header();
  r.stamp();                                  // map_load_time
  const resolution = r.float32(), width = r.uint32(), height = r.uint32();
  const [px, py, pz, qx, qy, qz, qw] = [0, 0, 0, 0, 0, 0, 0].map(() => r.float64());
  const data = r.int8Seq();
  return { header, info: { resolution, width, height, origin: { position: { x: px, y: py, z: pz },
    orientation: { x: qx, y: qy, z: qz, w: qw } } }, data };
}

/** std_msgs/Float32MultiArray -> {labels: dim labels, sizes, data}. @param {Uint8Array} b */
export function decodeFloat32MultiArray(b) {
  const r = new CdrReader(b);
  const nd = r.uint32();
  const dims = [];
  for (let i = 0; i < nd; i++) dims.push({ label: r.string(), size: r.uint32(), stride: r.uint32() });
  r.uint32();                                 // data_offset
  return { dims, data: r.float32Seq() };
}
