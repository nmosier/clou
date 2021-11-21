; ModuleID = 'stl6.c'
source_filename = "stl6.c"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "arm64-apple-macosx12.0.0"

@array_size = dso_local global i32 16, align 4
@publicarray = dso_local global [16 x i8] c"\01\02\03\04\05\06\07\08\09\0A\0B\0C\0D\0E\0F\10", align 1
@publicarray2 = dso_local global <{ i8, [131071 x i8] }> <{ i8 20, [131071 x i8] zeroinitializer }>, align 1
@secretarray = dso_local global [16 x i8] c"\0A\15 +6ALWbmny\84\8F\9A\A5", align 1
@temp = dso_local global i8 0, align 1
@case6_idx = dso_local global i32 0, align 4
@case6_array = dso_local global [2 x i8*] [i8* getelementptr inbounds ([16 x i8], [16 x i8]* @secretarray, i32 0, i32 0), i8* getelementptr inbounds ([16 x i8], [16 x i8]* @publicarray, i32 0, i32 0)], align 8

; Function Attrs: noinline nounwind optnone ssp uwtable
define dso_local void @case_6(i32 %0) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  %4 = alloca i8, align 1
  store i32 %0, i32* %2, align 4
  %5 = load i32, i32* %2, align 4
  %6 = load i32, i32* @array_size, align 4
  %7 = sub i32 %6, 1
  %8 = and i32 %5, %7
  store i32 %8, i32* %3, align 4
  store i32 1, i32* @case6_idx, align 4
  %9 = load i32, i32* @case6_idx, align 4
  %10 = zext i32 %9 to i64
  %11 = getelementptr inbounds [2 x i8*], [2 x i8*]* @case6_array, i64 0, i64 %10
  %12 = load i8*, i8** %11, align 8
  %13 = load i32, i32* %3, align 4 ; IGNORE
  %14 = zext i32 %8 to i64
  %15 = getelementptr inbounds i8, i8* %12, i64 %14
  %16 = load i8, i8* %15, align 1
  store i8 %16, i8* %4, align 1
  %17 = load i8, i8* %4, align 1
  %18 = zext i8 %17 to i32
  %19 = mul nsw i32 %18, 512
  %20 = sext i32 %19 to i64
  %21 = getelementptr inbounds [131072 x i8], [131072 x i8]* bitcast (<{ i8, [131071 x i8] }>* @publicarray2 to [131072 x i8]*), i64 0, i64 %20
  %22 = load i8, i8* %21, align 1
  %23 = zext i8 %22 to i32
  %24 = load volatile i8, i8* @temp, align 1
  %25 = zext i8 %24 to i32
  %26 = and i32 %25, %23
  %27 = trunc i32 %26 to i8
  store volatile i8 %27, i8* @temp, align 1
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
