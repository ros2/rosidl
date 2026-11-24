# Native shared-ownership bridge

Native backend extensions can return a Python `Buffer` using the private
`rosidl_buffer._share_buffer_from_capsule(capsule)` helper. It is also available
from `rosidl_buffer._rosidl_buffer_py`, but is excluded from `__all__`.

The capsule contract is:

- The exact name is `rosidl_buffer.shared_buffer` (no version suffix).
- The payload points to a heap-allocated
  `std::shared_ptr<rosidl::Buffer<uint8_t>>` holder containing a nonempty pointer.
- The capsule owns that holder. Its destructor deletes the holder, releasing
  that shared ownership reference.
- The name must remain valid for the capsule's entire lifetime; use static
  storage, such as a string literal.
- The receiver validates the capsule and exact name, then copies the existing
  `shared_ptr` into its wrapper. It never consumes, moves from, or deletes the
  holder, and never creates a new control block from the raw buffer pointer.

Repeated calls with one capsule create distinct Python wrappers sharing the
same native buffer and control block. The capsule, wrappers, and other native
shared owners can be destroyed in any order. The last owner destroys the buffer.
If Python result construction fails, the temporary wrapper releases only its
own reference.

Producers should convert their `unique_ptr` to a `shared_ptr`, allocate the
holder under RAII protection, construct an owning capsule, and release the
holder's RAII guard only after capsule construction succeeds. Keep the capsule
alive during the helper call. All failures, including import, attribute lookup,
argument construction, and receiver exceptions, then retain a responsible owner.
Do not pass a released raw pointer and attempt to recover ownership on failure:
the receiver may already have adopted and destroyed it.

This is an agreed **native-extension contract requiring compatible C++ ABIs**:
producer and receiver must agree on the `rosidl::Buffer<uint8_t>` definition,
`std::shared_ptr` representation, standard library, and compiler ABI settings.
Capsule name validation cannot verify arbitrary pointer contents or compiler
compatibility. Python's capsule API does not make this payload ABI-independent.
Only pass capsules from trusted, compatible native extensions.

`_take_buffer_from_ptr()` and `_get_buffer_ptr()` remain unchanged for existing
consumers. The new bridge does not change the `Buffer` wrapper layout or public
methods. Native producers must explicitly migrate to the new helper to gain
exception-safe shared ownership.
