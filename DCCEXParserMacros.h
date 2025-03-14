// Macro for printing a single argument
#define ZZARG(arg) \
  auto arg=p[_xix]; (void)arg;\
  if ( #arg[0]<='Z' && p[_xix]!=CONCAT(#arg,_hk)) break; \
  _xix++;

#define FOR_EACH_IMPL_1(x) ZZARG(x)
#define FOR_EACH_IMPL_2(x, ...) ZZARG(x) FOR_EACH_IMPL_1(__VA_ARGS__)
#define FOR_EACH_IMPL_3(x, ...) ZZARG(x) FOR_EACH_IMPL_2(__VA_ARGS__)
#define FOR_EACH_IMPL_4(x, ...) ZZARG(x) FOR_EACH_IMPL_3(__VA_ARGS__)
#define FOR_EACH_IMPL_5(x, ...) ZZARG(x) FOR_EACH_IMPL_4(__VA_ARGS__)
#define FOR_EACH_IMPL_6(x, ...) ZZARG(x) FOR_EACH_IMPL_5(__VA_ARGS__)
// Add more levels if needed...

// Step 1: Count the number of arguments
#define FOR_EACH_NARG(...) FOR_EACH_NARG_(__VA_ARGS__, 6, 5, 4, 3, 2, 1)
#define FOR_EACH_NARG_(_1, _2, _3, _4, _5, _6,N, ...) N

// Step 2: Force proper expansion (extra indirection to resolve `##`)
#define EXPAND(x) x
#define CONCAT(a, b) a##b

// Step 3: Dispatch to the correct implementation
#define FOR_EACH_IMPL(count, ...) EXPAND(CONCAT(FOR_EACH_IMPL_, count))(__VA_ARGS__)

// Step 4: Main macro to process arguments
#define FOR_EACH(...) FOR_EACH_IMPL(FOR_EACH_NARG(__VA_ARGS__), __VA_ARGS__)

#define ZZ(op,...) return true; }\
if (opcode==#op[0] && params==FOR_EACH_NARG(__VA_ARGS__)) for (auto _xix=0;;) { \
  FOR_EACH(__VA_ARGS__)

#define ZZBEGIN if (false) {
#define ZZEND return true; } return false;
#define CHECK(x) if (!(x)) return false;
#define EXPECT_CALLBACK CHECK(stashCallback(stream, p, ringStream)

