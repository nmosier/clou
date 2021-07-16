char arr[256];

char f(char i) {
   while (i != 0) {
      i = arr[i];
   }
   return i;
}
