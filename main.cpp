#include <cstdio>
#include "inputs.h"
#include "mq_enc.h"

int main() {
  uint8_t buf[8192];
  mq_encoder mq_enc(buf);
  mq_enc.init_coder();
  mq_enc.init_states_for_all_context();
  int32_t length = 7666;
  for (int i = 0; i < length; ++i) {
    mq_enc.encode(input_data[i][0], input_data[i][1]);
  }
  mq_enc.show();
  /*
   この入力を与えた場合，mq_encoderの内部状態は
   C = 55750336
   A = 36866
   t = 1
   T = 85
   L(出力バイト列の長さ) = 830
   出力バイト列 = [
     0,   133, 193, 50,  120, 231, 161, 95,  153, 105, 140, 193, 234, 27,  62,  156, 24,  188, 192, 141,
   137, 167, 50,  170, 72,  131, 235, 47,  204, 120, 171, 29,  43,  58,  167, 75,  2,   214, 63,  201, 33,
   166, 59,  153, 30,  135, 247, 159, 84,  180, 58,  74,  232, 62,  149, 41,  20,  92,  72,  156, 241, 88,
   124, 109, 203, 156, 51,  17,  203, 195, 221, 188, 37,  2,   187, 235, 43,  111, 235, 139, 62,  115, 206,
   135, 175, 202, 102, 84,  88,  190, 189, 13,  14,  240, 77,  12,  162, 155, 183, 209, 107, 99,  150, 182,
   230, 5,   37,  67,  133, 67,  151, 5,   141, 67,  137, 174, 166, 201, 120, 4,   247, 119, 52,  18,  183,
   160, 255, 39,  75,  89,  99,  5,   78,  119, 47,  1,   253, 110, 179, 189, 174, 74,  189, 78,  236, 107,
   185, 68,  211, 168, 191, 242, 48,  88,  210, 146, 24,  235, 103, 177, 202, 89,  40,  76,  178, 55,  32,
   10, 42,  159, 239, 138, 249, 166, 12,  105, 216, 9,   105, 155, 118, 21,  101, 16,  86,  27,  65,  92,
   108, 137, 85,  56,  137, 130, 18,  233, 14,  95,  1,   5,   210, 64,  209, 132, 70,  49,  107, 91,  204,
   27, 153, 77,  245, 209, 253, 130, 30,  150, 15,  25,  9,   146, 37,  15,  225, 225, 238, 32,  178, 232,
   211, 89,  170, 110, 121, 39,  246, 166, 167, 17,  44,  242, 218, 91,  71,  97,  147, 213, 86,  171, 69,
   33, 73,  188, 202, 92,  255, 10,  9,   244, 226, 200, 170, 1,   235, 98,  109, 243, 185, 31,  231, 60,
   231, 156, 111, 167, 128, 183, 26,  203, 152, 227, 93,  226, 135, 195, 146, 106, 159, 192, 120, 216, 162,
   164, 36,  2,   133, 97,  48,  119, 3,   55,  171, 120, 77,  11,  164, 38,  36,  12,  199, 138, 91,  57,
   109, 199, 83,  101, 250, 130, 226, 207, 218, 21,  132, 47,  186, 112, 36,  132, 116, 234, 14,  245, 34,
   32, 17,  83,  173, 92,  59,  78,  24,  204, 219, 146, 235, 102, 115, 12,  28,  22,  111, 15,  63,  188,
   47, 153, 105, 250, 251, 89,  147, 117, 242, 127, 171, 197, 91,  221, 49,  83,  34,  114, 232, 71,  120,
   126, 74,  155, 76,  166, 227, 17,  202, 215, 47,  238, 40,  196, 236, 68,  235, 19,  144, 254, 78,  126,
   122, 119, 35,  237, 98,  144, 119, 199, 15,  141, 104, 216, 213, 32,  192, 161, 123, 83,  200, 176, 224,
   149, 237, 153, 246, 137, 16,  213, 74,  50,  245, 127, 100, 102, 22,  10,  181, 7,   42,  65,  221, 126,
   179, 38,  57,  49,  151, 122, 248, 163, 207, 234, 124, 187, 195, 246, 236, 11,  135, 122, 189, 74,  110,
   244, 227, 145, 149, 236, 126, 3,   166, 126, 190, 153, 51,  202, 202, 5,   240, 222, 162, 2,   171, 243,
   254, 240, 38,  220, 114, 236, 209, 189, 23,  79,  63,  19,  166, 68,  16,  133, 76,  17,  214, 221, 235,
   61, 88,  59,  153, 89,  204, 250, 106, 151, 45,  25,  39,  202, 42,  217, 129, 168, 131, 32,  153, 125,
   134, 68,  168, 230, 246, 126, 239, 255, 54,  59,  215, 186, 174, 51,  62,  235, 6,   177, 23,  244, 34,
   211, 40,  101, 38,  52,  234, 92,  183, 9,   29,  176, 231, 238, 117, 50,  184, 39,  211, 31,  44,  240,
   69, 149, 80,  149, 73,  160, 157, 39,  165, 112, 39,  186, 62,  221, 111, 88,  14,  8,   147, 216, 42,
   37, 30,  133, 49,  130, 85,  101, 122, 214, 231, 233, 95,  28,  152, 177, 151, 9,   142, 3,   21,  250,
   50, 252, 23,  60,  183, 99,  77,  53,  106, 162, 75,  133, 75,  82,  102, 188, 142, 229, 237, 39,  98,
   200, 138, 167, 31,  144, 74,  138, 230, 11,  122, 34,  234, 44,  119, 252, 93,  192, 54,  74,  211, 59,
   227, 163, 27,  95,  246, 180, 89,  71,  249, 164, 79,  81,  201, 190, 179, 96,  232, 196, 12,  185, 120,
   102, 15,  115, 160, 180, 188, 180, 45,  13,  57,  51,  17,  63,  8,   193, 169, 194, 6,   151, 48,  98,
   96, 78,  115, 77,  154, 39,  220, 237, 56,  197, 24,  185, 90,  198, 222, 242, 162, 250, 33,  137, 242,
   194, 6,   144, 193, 226, 77,  139, 119, 30,  209, 223, 16,  82,  147, 239, 221, 198, 29,  32,  163, 13,
   69, 16,  8,   5,   6,   97,  206, 243, 233, 8,   97,  161, 148, 1,   44,  175, 187, 145, 254, 142, 32,
   212, 23,  215, 49,  145, 90,  34,  80,  10,  14,  79,  48,  110, 76,  71,  38,  160, 101, 68,  228, 195,
   212, 231, 112, 43,  227, 195, 246, 222, 32,  35,  82,  58,  50,  76,  94,  170, 134, 181, 114, 7,   183,
   95, 60,  82,  191, 47,  167, 217, 236, 239, 171, 5,   30,  246, 48,  168, 174, 128, 238, 40,  69,  224,
   136, 190, 243, 229, 3,   150, 8,   208, 141, 253, 250, 196
   ]
   になるはず
  */
}