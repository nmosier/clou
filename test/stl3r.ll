; ModuleID = 'stl3.c'
source_filename = "stl3.c"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "arm64-apple-macosx12.0.0"

@array_size = dso_local global i32 16, align 4
@publicarray = dso_local global [16 x i8] c"\01\02\03\04\05\06\07\08\09\0A\0B\0C\0D\0E\0F\10", align 1
@publicarray2 = dso_local global <{ i8, [131071 x i8] }> <{ i8 20, [131071 x i8] zeroinitializer }>, align 1
@secretarray = dso_local global [16 x i8] c"\0A\15 +6ALWbmny\84\8F\9A\A5", align 1
@temp = dso_local global i8 0, align 1

; Function Attrs: noinline nounwind optnone ssp uwtable
define dso_local void @case_3(i32 %0) #0 {
  %2 = alloca i32, align 4
  %3 = add i1 0, 0
  store i32 %0, i32* %2, align 4
  %4 = load i32, i32* %2, align 4
  %5 = load i32, i32* @array_size, align 4
  %6 = sub i32 %5, 1
  %7 = and i32 %4, %6
  ; store i32 %7, i32* %3, align 4
  %8 = add i1 0, 0
  %9 = zext i32 %7 to i64
  %10 = getelementptr inbounds [16 x i8], [16 x i8]* @publicarray, i64 0, i64 %9
  %11 = load i8, i8* %10, align 1
  %12 = zext i8 %11 to i32
  %13 = mul nsw i32 %12, 512
  %14 = sext i32 %13 to i64
  %15 = getelementptr inbounds [131072 x i8], [131072 x i8]* bitcast (<{ i8, [131071 x i8] }>* @publicarray2 to [131072 x i8]*), i64 0, i64 %14
  %16 = load i8, i8* %15, align 1
  %17 = zext i8 %16 to i32
  %18 = load volatile i8, i8* @temp, align 1
  %19 = zext i8 %18 to i32
  %20 = and i32 %19, %17
  %21 = trunc i32 %20 to i8
  store volatile i8 %21, i8* @temp, align 1
  ret void
}

attributes #0 = { noinline nounwind optnone ssp uwtable "disable-tail-calls"="false" "frame-pointer"="non-leaf" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="apple-a12" "target-features"="+aes,+crc,+crypto,+fp-armv8,+fullfp16,+lse,+neon,+ras,+rcpc,+rdm,+sha2,+v8.3a,+zcm,+zcz" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.module.flags = !{!0, !1, !2, !3, !4, !5}
!llvm.ident = !{!6}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 1, !"branch-target-enforcement", i32 0}
!2 = !{i32 1, !"sign-return-address", i32 0}
!3 = !{i32 1, !"sign-return-address-all", i32 0}
!4 = !{i32 1, !"sign-return-address-with-bkey", i32 0}
!5 = !{i32 7, !"PIC Level", i32 2}
!6 = !{!"Homebrew clang version 12.0.1"}
