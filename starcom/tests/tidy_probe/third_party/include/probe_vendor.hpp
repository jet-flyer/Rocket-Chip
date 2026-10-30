// probe_vendor.hpp: stands for a vendored header (tl::expected, pico-sdk).
// It holds a defect on purpose. The filter must not report it.
inline unsigned probeVendor() { return 9u; }
