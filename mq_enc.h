#pragma once
#include <cstdint>
class mq_encoder {
 private:
  uint16_t A;
  uint8_t t;
  // Lower-bound interval
  uint32_t C;
  // Temporary byte register
  uint8_t T;
  // position in byte-stream
  uint32_t L;
  // start position in byte-stream
  uint32_t L_start;
  // position of current codeword segment boundary
  uint32_t Lmax;
  // pointer to byte-stream buffer
  uint8_t *const byte_buffer;
  // position of next element of byte_buffer[]
  int32_t buf_next;
  // position of beginning of byte_buffer[]
  int32_t buf_start;
  // dynamic table for context
  uint16_t dynamic_table[2][19];
  // static table for state transition
  const uint16_t static_table[4][47];

 public:
  // constructor
  mq_encoder(uint8_t *const buf);
  // initialize mq_encoder
  void init_coder();
  // initialize all internal states
  void init_states_for_all_context();
  // put a byte to byte-buffer
  void put_byte();
  // determine if bit-stuffing is needed
  void transfer_byte();
  // encoding function which takes input symbol x, and corresponding context label
  void encode(uint8_t &x, uint8_t label);
  // debug function
  void show();
};