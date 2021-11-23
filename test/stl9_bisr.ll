; ModuleID = 'stl9_bis.c'
source_filename = "stl9_bis.c"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "arm64-apple-macosx12.0.0"

@array_size = dso_local global i32 16, align 4
@publicarray = dso_local global [16 x i8] c"\01\02\03\04\05\06\07\08\09\0A\0B\0C\0D\0E\0F\10", align 1
@publicarray2 = dso_local global <{ i8, [131071 x i8] }> <{ i8 20, [131071 x i8] zeroinitializer }>, align 1
@secretarray = dso_local global [16 x i8] c"\0A\15 +6ALWbmny\84\8F\9A\A5", align 1
@temp = dso_local global i8 0, align 1

; Function Attrs: noinline nounwind optnone ssp uwtable
define dso_local void @case_9_bis(i32 %0) #0 {
  %2 = alloca i32, align 4
  %3 = alloca i32, align 4
  %4 = alloca i32, align 4
  store i32 %0, i32* %2, align 4
  %5 = load i32, i32* %2, align 4
  %6 = load i32, i32* @array_size, align 4
  %7 = sub i32 %6, 1
  %8 = and i32 %5, %7
  store i32 %8, i32* %3, align 4 ; IGNORE: *%3 = %8
  %9 = load i32, i32* %3, align 4 ; IGNORE: %9 -> %8
  %10 = zext i32 %8 to i64
  %11 = getelementptr inbounds [16 x i8], [16 x i8]* @secretarray, i64 0, i64 %10
  store i8 0, i8* %11, align 1
  store i32 0, i32* %4, align 4
  br label %12

12:                                               ; preds = %21, %1
  %13 = phi i32 [ 0, %1 ], [ %23, %21 ]
  ; %13 = load i32, i32* %4, align 4
  %14 = icmp ult i32 %13, 10
  br i1 %14, label %15, label %24

15:                                               ; preds = %12
  %16 = load i32, i32* %4, align 4 ; IGNORE: replace %16 -> %13
  %17 = load volatile i8, i8* @temp, align 1
  %18 = zext i8 %17 to i32
  %19 = and i32 %18, %13
  %20 = trunc i32 %19 to i8
  store volatile i8 %20, i8* @temp, align 1
  br label %21

21:                                               ; preds = %15
  %22 = load i32, i32* %4, align 4
  %23 = add i32 %22, 1
  ; store i32 %23, i32* %4, align 4
  br label %12, !llvm.loop !7

24:                                               ; preds = %12
  %25 = load i32, i32* %3, align 4 ; IGNORE: replace %25 -> %8
  %26 = zext i32 %8 to i64
  %27 = getelementptr inbounds [16 x i8], [16 x i8]* @secretarray, i64 0, i64 %26
  %28 = load i8, i8* %27, align 1
  %29 = zext i8 %28 to i32
  %30 = mul nsw i32 %29, 512
  %31 = sext i32 %30 to i64
  %32 = getelementptr inbounds [131072 x i8], [131072 x i8]* bitcast (<{ i8, [131071 x i8] }>* @publicarray2 to [131072 x i8]*), i64 0, i64 %31
  %33 = load i8, i8* %32, align 1
  %34 = zext i8 %33 to i32
  %35 = load volatile i8, i8* @temp, align 1
  %36 = zext i8 %35 to i32
  %37 = and i32 %36, %34
  %38 = trunc i32 %37 to i8
  store volatile i8 %38, i8* @temp, align 1
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
!7 = distinct !{!7, !8}
!8 = !{!"llvm.loop.mustprogress"}
