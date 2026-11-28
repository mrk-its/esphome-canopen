# Writable strings in the object dictionary

Status: design only (2026-11-28), not implemented. It came up in Copilot's review of PR 5 (esphome/esphome#23440,
"PR 5 as cut" in [canopennode.md](../canopennode.md)): `read_od()` / `write_od()` don't do CANopenNode's `ODA_STR`
string handling. PR 5 only documents that; writable strings already work with OD handlers (below), and a helper can
come in a later PR when something needs it.

## Strings in the hub today

Three kinds, none of them uses `ODA_STR`:

1. **Constant read-only strings: `ObjectDictionary::add_string(index, const char *str)`** (`od.h`, `od.cpp`, on `dev`
   since PR 3, esphome/esphome#23299). A read-only VAR at sub-index 0 pointing at the caller's string (not copied, must
   outlive the hub), size `strlen(str)` fixed when it's added, no attribute argument. Without `ODA_STR` the SDO server
   indicates the exact size; strings of up to 4 bytes go expedited. An empty string is one zero byte: CANopenNode
   aborts zero-length uploads. Used for `0x1008` device name, `0x1009` hardware version (if set), `0x100A` software
   version (`device_info.cpp`).
2. **Strings made on the fly: a domain entry with an `od_read()` handler.** The entry has no storage and length 0
   (`{sub, OD_READ, nullptr, 0}` / `add_var(index, OD_READ, nullptr, 0)`); the handler produces the text and calls
   `OdStream::set_size(len)` on its first call (`offset() == 0`), so the client gets the exact size. `canopen_entities`
   does this for the entity metadata at `0x2000 + 16 * N`, sub-indices 1-4 (`MappedEntity::od_read()`); remote logs'
   Log Data `0x3100:1` is a domain the same way.
3. **Writable strings: a domain entry with an `od_write()` handler.** Possible with PR 5's API, nothing uses it yet.
   The handler gets the download in chunks (`offset()` advances, the handler moves it with `set_offset()`), `size()` is
   0 until the last chunk, which carries the total. `write_od()` on a domain passes `len` as the total, in one call.
   Tested over SDO since PR 5's review: `SdoDownloadIntoADomainHandler` in `loopback_test.cpp`.

Our attributes are `OD_READ` / `OD_WRITE` / `OD_READ_WRITE` only (`od.h`). `ODA_STR` is reachable only as CANopenNode's
raw constant (`od.h` includes `<CANopen.h>`).

## CANopenNode's `ODA_STR`

Bit `0x80` of an OD sub-entry's attribute (`301/CO_ODinterface.h`): "Shorter value, than specified variable size, may
be written to the variable. SDO write will fill remaining memory with zeroes. Attribute is used for VISIBLE_STRING and
UNICODE_STRING." Only the SDO server reads it (`301/CO_SDOserver.c`):

- **Download:** without `ODA_STR` the size must equal the entry's (DATA_SHORT / DATA_LONG abort otherwise). With it a
  shorter value is accepted: the server appends a zero terminator, 2 bytes (UNICODE_STRING), or 1 if only one byte of
  room is left, and shortens the stream length for that write (segmented ~line 421, expedited ~line 729). Despite the
  header, it writes only those 1-2 zeros: the old bytes behind them stay. A value of exactly the entry's size gets no
  terminator.
- **Upload:** without `ODA_STR` the server indicates the entry's size and sends all of it. With it the size isn't
  indicated (segmented initiate with no size, ~line 691) and the server sends up to the first zero byte (strlen per
  chunk, ~line 542), at least 1 byte (zero length isn't allowed).

So it's meant for writable strings in a fixed buffer of a maximum length.

## Proposal: `OdString<N>`, a domain with a handler

One class owns the buffer and is its own OD handler; a component never sees chunks, offsets or terminators.

### Component side

```cpp
#include "esphome/components/canopen/canopen.h"

class MyComponent : public Component {
 public:
  explicit MyComponent(canopen::CANopenComponent *hub) : hub_(hub) {}

  // from codegen (before the hub's setup())
  void add_od_entries() {
    this->label_.set("kitchen");  // initial value
    this->label_.set_on_write(
        [](void *context, const char *value) { static_cast<MyComponent *>(context)->apply_label_(value); }, this);
    this->hub_->get_od().add_string(0x2010, &this->label_);  // read / write over SDO and write_od()
  }

  void some_event_() {
    this->label_.set("living room");  // changed locally; the next SDO read returns it
  }

 protected:
  void apply_label_(const char *value) { ESP_LOGI(TAG, "label: %s", value); }

  canopen::CANopenComponent *hub_;
  canopen::OdString<32> label_;  // up to 32 characters, 33 bytes of storage
};
```

- `OdString<N>`: up to `N` characters, the storage inside the object.
- `set()` / `c_str()` / `size()`: change and read the value locally.
- `set_on_write()`: called once a whole new value has arrived (SDO download from another node, or `write_od()`), not
  for `set()`. A plain function pointer with a `void *context`, like `SDOCallback` / `FrameCallback`. Instead, a
  component can derive from `OdString<N>` and override `on_write()`, or shadow `od_write()` (call the base, act on
  `OK`): `set_object<T>()` builds the trampoline for the type it's given and calls `T::od_write()` directly, so the
  derived version wins as long as `set_object()` gets the derived type.
- `add_string(index, &obj)`: an overload next to `add_string(index, const char *)`, which stays for constant read-only
  strings.

The plain handler API already does all of this (a domain entry plus an `OdObject` with `od_read()` / `od_write()`); the
helper only hides the boilerplate: the length limit, the chunk offsets, the terminator, the size on reads, the
callback.

### Implementation draft (`od_handler.h`)

```cpp
template<size_t N> class OdString : public OdObject {
 public:
  using WriteCallback = void (*)(void *context, const char *value);

  void set(const char *value) {
    this->len_ = std::min(std::strlen(value), N);
    std::memcpy(this->buf_.data(), value, this->len_);
    this->buf_[this->len_] = '\0';
  }
  const char *c_str() const { return this->buf_.data(); }
  size_t size() const { return this->len_; }
  void set_on_write(WriteCallback callback, void *context) {
    this->callback_ = callback;
    this->context_ = context;
  }

  // the OD handler: a domain entry, so any length up to N, exact size on reads
  OdResult od_write(OdStream stream, const void *buf, size_t count, size_t *done) {
    if (stream.offset() + count > N)
      return OdResult::OD_RESULT_DATA_LONG;  // longer than N: SDO abort, old value kept
    std::memcpy(this->staging_.data() + stream.offset(), buf, count);
    stream.set_offset(stream.offset() + count);
    *done = count;
    if (stream.size() == 0)
      return OdResult::OD_RESULT_PARTIAL;  // more chunks follow
    // complete: take it over in one step, so nobody sees a half-written value
    this->len_ = stream.offset();
    std::memcpy(this->buf_.data(), this->staging_.data(), this->len_);
    this->buf_[this->len_] = '\0';
    this->on_write(this->c_str());
    return OdResult::OD_RESULT_OK;
  }
  OdResult od_read(OdStream stream, void *buf, size_t count, size_t *done) {
    if (stream.offset() == 0)
      stream.set_size(this->len_ > 0 ? this->len_ : 1);  // empty: one zero byte, as add_string()
    size_t n = std::min(count, stream.size() - stream.offset());
    std::memcpy(buf, this->buf_.data() + stream.offset(), n);
    stream.set_offset(stream.offset() + n);
    *done = n;
    return stream.offset() < stream.size() ? OdResult::OD_RESULT_PARTIAL : OdResult::OD_RESULT_OK;
  }

 protected:
  virtual void on_write(const char *value) {
    if (this->callback_ != nullptr)
      this->callback_(this->context_, value);
  }

  std::array<char, N + 1> buf_{};
  std::array<char, N> staging_{};  // a download in chunks lands here until it is complete
  size_t len_{0};
  WriteCallback callback_{nullptr};
  void *context_{nullptr};
};

// in ObjectDictionary
template<size_t N> void add_string(uint16_t index, OdString<N> *str, uint8_t attr = OD_READ_WRITE) {
  this->add_var(index, attr, nullptr, 0);  // a domain: the handler has the storage
  this->set_object(index, str);
}
```

Design notes:

- **Domain entry:** no fixed length, so writes of any length up to `N` are accepted and reads report the current
  length exactly, also after `set()` (plain `add_string()` fixes the size when it's added).
- **Staging buffer:** a download in several chunks collects in `staging_` and is copied over only when complete, so a
  read in between or an aborted transfer never sees half a string; costs `N` bytes. Cheaper variant: no staging, and
  require that `N` fits the SDO server's buffer (`CO_CONFIG_SDO_SRV_BUFFER_SIZE`: CANopenNode's default 32 now, 900
  with `sdo_block_size` / PDOs, PRs 6-7), so the whole string arrives in one call.
- **`on_write()` virtual:** only `od_read()` / `od_write()` go through the trampoline, so the virtual costs one call
  per complete write. Without it, shadowing `od_write()` works as well.
- **No `ODA_STR`:** no terminator over the bus, the exact size indicated on reads, `write_od()` the same as SDO.

## Compared with `ODA_STR`

The `ODA_STR` equivalent: a fixed buffer with the string attribute, plus a small handler for the notification:

```cpp
char label[33] = "kitchen";  // 32 characters + a terminator that always stays
od.add_var(0x2010, OD_READ_WRITE | ODA_STR, label, 32);
od.set_object(0x2010, &label_handler);  // only for the notification: write_original(), then the callback on OK
```

| | `ODA_STR` (fixed buffer) | Domain + handler (`OdString<N>`) |
|---|---|---|
| Shorter write over SDO | accepted, the server appends 1-2 zeros | accepted, the handler terminates it |
| Write longer than the maximum | DATA_LONG abort | DATA_LONG abort |
| Buffer after a shorter write | old bytes remain behind the 1-2 zeros | a fresh copy |
| Upload size | not indicated (segmented, no size), cut at the first zero byte | exact length indicated |
| Empty string | 1 zero byte sent | same (chosen to match `add_string()`) |
| Write in several chunks (longer than the server buffer) | goes straight into the storage; an aborted transfer leaves old and new mixed | staged, copied over when complete |
| `write_od()` / `read_od()` | differ from SDO: needs Copilot's fix (~30 lines + tests) | already the same as SDO |
| Change notification | needs a handler anyway (`write_original()`, act on `OK`) | built in (`set_on_write()`) |
| Local `set()` | `strcpy` into the buffer; reads follow (cut at the terminator) | `set()`; reads follow (exact size) |
| Sub-index of a record / array | yes: the attribute is per sub-index, no handler needed | no: `set_object()` is per index, a record mixing it with other subs needs its own dispatching handler |
| RAM | N+1 | 2N+1 with staging (N+1 without, if N fits the server buffer) |
| Code | none new in the library (the server's `ODA_STR` branches are compiled anyway), a small notification handler | ~60-line template |
| Familiarity | the CANopenNode way; ODs generated from an EDS mark VISIBLE_STRING like this | our own pattern; over SDO a string is just bytes (no type on the wire), clients don't notice |

`ODA_STR`:

- Pros: CANopenNode's own mechanism, nearly no new code; works per sub-index (a string next to numbers in one record,
  no handler), which `OdString` can't easily match; the least RAM.
- Cons: no upload size; a string longer than the server buffer is written in place chunk by chunk, so an aborted or
  half-finished write is visible; old bytes behind the terminator; `write_od()` / `read_od()` need the `ODA_STR`
  handling to match SDO; the terminator is part of the protocol (a value of exactly N characters gets none), so the
  storage needs a byte more than the OD length, easy to get wrong by hand.

Domain + handler:

- Pros: exact sizes both ways; the value changes in one step; local access and SDO the same, no special cases; the
  limit and the notification in one place; works with PR 5's API, nothing new needed in the hub.
- Cons: one string per OD index (or a dispatching handler); our own code to maintain; up to N bytes more RAM
  (staging).

## Recommendation

- For a component-facing API, `OdString<N>` + `add_string(index, &obj)` on a domain: no protocol quirks leak out, and
  it needs nothing beyond PR 5.
- `ODA_STR` earns its place if strings must be sub-indices of records next to other values (CiA profiles that define
  string subs in records). The helper could then get an `ODA_STR`-backed variant, which needs the `read_od()` /
  `write_od()` `ODA_STR` handling Copilot asked for (shorter writes with the terminator, reads up to the terminator,
  including a one-byte look-ahead at the caller's buffer boundary).
- Neither in PR 5. Its own small PR once a component needs writable strings.

## Open points

- Empty string writes: an SDO download of 0 bytes, and `write_od(index, sub, data, 0)` on a domain, where `size()` is
  0 so the handler returns `PARTIAL` and `write()` turns that into DATA_SHORT. Decide whether an empty write must
  work, and handle it in `ObjectDictionary::write()` / the handler.
- Whether `set()` should truncate silently at `N` (as drafted) or report it.
- Tests: SDO round trip through `LocalPair` (short, exactly N, longer than N -> DATA_LONG with the old value kept,
  longer than the server buffer -> staged), `write_od()` / `read_od()`, the callback once per complete write and not
  for `set()`, an aborted transfer leaving the old value.
