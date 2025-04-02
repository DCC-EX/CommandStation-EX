
// Count the number of arguments
#define FOR_EACH_NARG(...) FOR_EACH_NARG_HELPER(__VA_ARGS__,8,7, 6,5,4, 3, 2, 1, 0)
#define FOR_EACH_NARG_HELPER(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N

// Step 2: Force proper expansion (extra indirection to resolve `##`)
#define EXPAND(x) x
#define CONCAT(a, b) a##b


#define ZZZ(_i,_arg) \
  if ( #_arg[0]<='Z' && p[_i]!=CONCAT(#_arg,_hk)) break; \
  auto _arg=p[_i]; (void) _arg; 

// Each ZZ terminates the previous one 
#define ZPREP(op,count) return true; } if (opcode==#op[0] && params==count) for (;;) {
#define Z1(op)                      ZPREP(op,0)
#define Z2(op,_1)                   ZPREP(op,1) ZZZ(0,_1) 
#define Z3(op,_1,_2)                ZPREP(op,2) ZZZ(0,_1) ZZZ(1,_2)
#define Z4(op,_1,_2,_3)             ZPREP(op,3) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3)
#define Z5(op,_1,_2,_3,_4)          ZPREP(op,4) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4)
#define Z6(op,_1,_2,_3,_4,_5)       ZPREP(op,5) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5)
#define Z7(op,_1,_2,_3,_4,_5,_6)    ZPREP(op,6) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5) ZZZ(5,_6)
#define Z8(op,_1,_2,_3,_4,_5,_6,_7) ZPREP(op,7) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5) ZZZ(5,_6) ZZZ(6,_7)

#define ZRIP(count) CONCAT(Z,count)
#define ZZ(...) ZRIP(FOR_EACH_NARG(__VA_ARGS__))(__VA_ARGS__) DCCEXParser::matchedCommandFormat = F( #__VA_ARGS__);
 
#define ZZBEGIN if (false) {
#define ZZEND return true; } return false;
#define CHECK(x) if (!(x)) { DCCEXParser::checkFailedFormat=F(#x); return false;}
#define REPLY(format,...) StringFormatter::send(stream,F(format), ##__VA_ARGS__);
#define EXPECT_CALLBACK CHECK(stashCallback(stream, p, ringStream))

