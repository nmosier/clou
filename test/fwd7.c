
extern int array1[];
extern int array2[];
extern int temp;

void example_7(int j, int attacker_val) {
  unsigned i = 0;

  if (j < 16) {
     array1[j] = attacker_val;
  }
  
  for (; i < 16; ++i) {
    temp &= array2[array1[i]];
  }
}
