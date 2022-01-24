#include <cstdio>
#include "mq_enc.h"

mq_encoder::mq_encoder(uint8_t *const buf)
    : A(0),
      t(0),
      C(0),
      T(0),
      L(0),
      L_start(0),
      Lmax(0),
      byte_buffer(buf),
      buf_next(0),
      buf_start(0),
      dynamic_table{{}},
      static_table{
          {1,  2,  3,  4,  5,  38, 7,  8,  9,  10, 11, 12, 13, 29, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
           25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 45, 46},
          {1,  6,  9,  12, 29, 33, 6,  14, 14, 14, 17, 18, 20, 21, 14, 14, 15, 16, 17, 18, 19, 19, 20, 21,
           22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 46},
          {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0x5601, 0x3401, 0x1801, 0x0AC1, 0x0521, 0x0221, 0x5601, 0x5401, 0x4801, 0x3801, 0x3001, 0x2401,
           0x1C01, 0x1601, 0x5601, 0x5401, 0x5101, 0x4801, 0x3801, 0x3401, 0x3001, 0x2801, 0x2401, 0x2201,
           0x1C01, 0x1801, 0x1601, 0x1401, 0x1201, 0x1101, 0x0AC1, 0x09C1, 0x08A1, 0x0521, 0x0441, 0x02A1,
           0x0221, 0x0141, 0x0111, 0x0085, 0x0049, 0x0025, 0x0015, 0x0009, 0x0005, 0x0001, 0x5601}} {}
void mq_encoder::init_coder() {
  A         = 0x8000;
  C         = 0;
  t         = 12;
  T         = 0;
  buf_next  = -1;
  buf_start = 0;
}
void mq_encoder::init_states_for_all_context() {
  for (int i = 0; i < 19; ++i) {
    dynamic_table[0][i] = 0;
    dynamic_table[1][i] = 0;
  }
  dynamic_table[0][0]  = 4;
  dynamic_table[0][17] = 3;
  dynamic_table[0][18] = 46;
}
void mq_encoder::put_byte() {
  byte_buffer[L] = T;
  L++;
  buf_next++;
}
void mq_encoder::transfer_byte() {
  constexpr uint32_t C_partial_zero = 0xF807FFFF;
  constexpr uint32_t C_partial_mask = 0x7F80000;
  constexpr uint32_t C_msbs_mask    = 0xFF00000;
  constexpr uint32_t C_msbs_zero    = 0xF00FFFFF;
  constexpr uint32_t C_carry_mask   = 0x8000000;
  constexpr uint32_t C_carry_zero   = 0xF7FFFFFF;
  if (T == 0xFF) {  // can't propagate any carry past T; need bit stuff
    put_byte();
    T = static_cast<uint8_t>(((C & C_msbs_mask) >> 20) & 0xFF);
    C &= C_msbs_zero;
    t = 7;
  } else {
    T += static_cast<uint8_t>(((C & C_carry_mask) >> 27) & 0xFF);
    C &= C_carry_zero;
    put_byte();
    if (T == 0xFF) {  // decoder will see this as a bit stuff; need to act accordingly
      T = static_cast<uint8_t>(((C & C_msbs_mask) >> 20) & 0xFF);
      C &= C_msbs_zero;
      t = 7;
    } else {
      T = static_cast<uint8_t>(((C & C_partial_mask) >> 19) & 0xFF);
      C &= C_partial_zero;
      t = 8;
    }
  }
}
void mq_encoder::encode(uint8_t &x, uint8_t label) {
  uint16_t p_bar = static_table[3][dynamic_table[0][label]];
  uint8_t s_k    = dynamic_table[1][label];
  A -= p_bar;
  if (x == s_k) {       // coding an MPS
    if (A >= 0x8000) {  // no renormalization and hence no conditional exchange
      C += static_cast<uint32_t>(p_bar);
    } else {
      if (A < p_bar) {  // conditional exchange
        A = p_bar;
      } else {
        C += static_cast<uint32_t>(p_bar);
      }
      dynamic_table[0][label] = static_table[0][dynamic_table[0][label]];
      while (true) {
        A <<= 1;
        C <<= 1;
        t--;
        if (t == 0) {
          transfer_byte();
        }
        if (A >= 0x8000) {
          break;
        }
      }
    }
  } else {            // coding an LPS; renormalization is inevitable
    if (A < p_bar) {  // conditional exchange
      C += static_cast<uint32_t>(p_bar);
    } else {
      A = p_bar;
    }
    dynamic_table[1][label] ^= static_table[2][dynamic_table[0][label]];
    dynamic_table[0][label] = static_table[1][dynamic_table[0][label]];
    while (true) {
      A <<= 1;
      C <<= 1;
      t--;
      if (t == 0) {
        transfer_byte();
      }
      if (A >= 0x8000) {
        break;
      }
    }
  }
}
void mq_encoder::show() {
  printf("Internal states of MQencoder:\n");
  printf("\t A = 0x%04X\n", A);
  printf("\t t = 0x%02X\n", t);
  printf("\t C = 0x%08X\n", C);
  printf("\t T = 0x%02X\n", T);
  printf("\t length = %d\n", L);
  printf("byte_buffer[%d] =\n", L);
  for (int i = 0; i < L; ++i) {
    printf("%02X ", byte_buffer[i]);
  }
  printf("\n");
}