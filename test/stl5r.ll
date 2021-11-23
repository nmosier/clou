; ModuleID = 'stl5.c'
source_filename = "stl5.c"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "arm64-apple-macosx12.0.0"

@array_size = dso_local global i32 16, align 4
@publicarray = dso_local global [16 x i8] c"\01\02\03\04\05\06\07\08\09\0A\0B\0C\0D\0E\0F\10", align 1
@publicarray2 = dso_local global <{ i8, [131071 x i8] }> <{ i8 20, [131071 x i8] zeroinitializer }>, align 1
@secretarray = dso_local global [16 x i8] c"\0A\15 +6ALWbmny\84\8F\9A\A5", align 1
@temp = dso_local global i8 0, align 1
@case5_ptr = dso_local global i8* getelementptr inbounds ([16 x i8], [16 x i8]* @secretarray, i32 0, i32 0), align 8

; Function Attrs: noinline nounwind optnone ssp uwtable
define dso_local void @case_5(i32 %0) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4 ; IGNORE
  %4 = alloca i8, align 1
  store i32 %0, i32* %2, align 4
  %5 = load i32, i32* %2, align 4
  %6 = load i32, i32* @array_size, align 4
  %7 = sub i32 %6, 1
  %8 = and i32 %5, %7
  store i32 %8, i32* %3, align 4
  store i8* getelementptr inbounds ([16 x i8], [16 x i8]* @publicarray, i64 0, i64 0), i8** @case5_ptr, align 8
  %9 = load i8*, i8** @case5_ptr, align 8
  %10 = load i32, i32* %3, align 4 ; IGNORE
  %11 = zext i32 %8 to i64
  %12 = getelementptr inbounds i8, i8* %9, i64 %11
  %13 = load i8, i8* %12, align 1
  store i8 %13, i8* %4, align 1
  %14 = load i8, i8* %4, align 1
  %15 = zext i8 %14 to i32
  %16 = mul nsw i32 %15, 512
  %17 = sext i32 %16 to i64
  %18 = getelementptr inbounds [131072 x i8], [131072 x i8]* bitcast (<{ i8, [131071 x i8] }>* @publicarray2 to [131072 x i8]*), i64 0, i64 %17
  %19 = load i8, i8* %18, align 1
  %20 = zext i8 %19 to i32
  %21 = load volatile i8, i8* @temp, align 1
  %22 = zext i8 %21 to i32
  %23 = and i32 %22, %20
  %24 = trunc i32 %23 to i8
  store volatile i8 %24, i8* @temp, align 1
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
