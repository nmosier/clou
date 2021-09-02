define dso_local i32 @f(i32* %0, i1 %pred) #0 {
entry:
  br label %loop
  
loop:
  %a = load i32, i32* %0
  %b = add i32 %a, 1
  store i32 %b, i32* %0
  br i1 %pred, label %loop, label %exit

exit:
  ret i32 0
}
