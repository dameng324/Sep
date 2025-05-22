using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using static System.Runtime.CompilerServices.Unsafe;
using static nietras.SeparatedValues.SepDefaults;
using static nietras.SeparatedValues.SepParseMask;
using Vec = System.Runtime.Intrinsics.Vector128;
using VecUI16 = System.Runtime.Intrinsics.Vector128<ushort>;
using VecUI8 = System.Runtime.Intrinsics.Vector128<byte>;

namespace nietras.SeparatedValues;

// https://lemire.me/blog/2017/07/10/pruning-spaces-faster-on-arm-processors-with-vector-table-lookups/

sealed class SepParserVector128AdvSimdNrwCmpExtMsbTzcnt : ISepParser
{
    readonly char _separator;
    readonly VecUI8 _nls = Vec.Create(LineFeedByte);
    readonly VecUI8 _crs = Vec.Create(CarriageReturnByte);
    readonly VecUI8 _qts;
    readonly VecUI8 _sps;
    nuint _quoteCount = 0;

    public unsafe SepParserVector128AdvSimdNrwCmpExtMsbTzcnt(SepParserOptions options)
    {
        _separator = options.Separator;
        _sps = Vec.Create((byte)_separator);
        _qts = Vec.Create((byte)options.QuotesOrSeparatorIfDisabled);
    }

    // Parses 2 x char vectors e.g. 1 byte vector
    public int PaddingLength => VecUI8.Count;
    public int QuoteCount => (int)_quoteCount;

    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public void ParseColEnds(SepReaderState s)
    {
        Parse<int, SepColEndMethods>(s);
    }

    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public void ParseColInfos(SepReaderState s)
    {
        Parse<SepColInfo, SepColInfoMethods>(s);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void Parse<TColInfo, TColInfoMethods>(SepReaderState s)
        where TColInfo : unmanaged
        where TColInfoMethods : ISepColInfoMethods<TColInfo>
    {
        // Method should **not** call other non-inlined methods, since this
        // impacts code-generation severely.

        // Unpack instance fields
        var separator = _separator;
        var quoteCount = _quoteCount;
        // Use instance fields to force values into registers
        var nls = _nls; //Vec.Create(LineFeedByte);
        var crs = _crs; //Vec.Create(CarriageReturnByte);
        var qts = _qts; //Vec.Create(QuoteByte);
        var sps = _sps; //Vec.Create(_separator);

        // Unpack state fields
        var chars = s._chars;
        var charsIndex = s._charsParseStart;
        var charsEnd = s._charsDataEnd;
        var lineNumber = s._parsingLineNumber;
        var colInfos = s._colEndsOrColInfos;

        var colInfosLength = TColInfoMethods.IntsLengthToColInfosLength(colInfos.Length);

        chars.CheckPaddingAndIsZero(charsEnd, PaddingLength);
        SepArrayExtensions.CheckPadding(colInfosLength, s._parsingRowColCount + s._parsingRowColEndsOrInfosStartIndex, PaddingLength);
        A.Assert(charsIndex <= charsEnd);
        A.Assert(charsEnd <= (chars.Length - PaddingLength));

        ref var charsOriginRef = ref MemoryMarshal.GetArrayDataReference(chars);

        ref var colInfosRefOrigin = ref As<int, TColInfo>(ref MemoryMarshal.GetArrayDataReference(colInfos));
        ref var colInfosRef = ref Add(ref colInfosRefOrigin, s._parsingRowColEndsOrInfosStartIndex);
        ref var colInfosRefCurrent = ref Add(ref colInfosRefOrigin, s._parsingRowColCount + s._parsingRowColEndsOrInfosStartIndex);
        ref var colInfosRefEnd = ref Add(ref colInfosRefOrigin, colInfosLength);
        var colInfosStopLength = colInfosLength - VecUI8.Count - SepReaderState.ColEndsOrInfosExtraEndCount;
        ref var colInfosRefStop = ref Add(ref colInfosRefOrigin, colInfosStopLength);

        charsIndex -= VecUI8.Count;
    LOOPSTEP:
        charsIndex += VecUI8.Count;
    LOOPNOSTEP:
        if (charsIndex < charsEnd &&
            // If current is greater than or equal than "stop", then there is no
            // longer guaranteed space enough for next VecUI8.Count + next row start.
            !IsAddressLessThan(ref colInfosRefStop, ref colInfosRefCurrent))
        {
            ref var charsRef = ref Add(ref charsOriginRef, (uint)charsIndex);
            ref var byteRef = ref As<char, byte>(ref charsRef);
            var v0 = ReadUnaligned<VecUI16>(ref byteRef);
            var v1 = ReadUnaligned<VecUI16>(ref Add(ref byteRef, VecUI8.Count));

            var r0 = AdvSimd.ExtractNarrowingSaturateLower(v0);
            var r1 = AdvSimd.ExtractNarrowingSaturateUpper(r0, v1);
            var bytes = r1;

            var nlsEq = AdvSimd.CompareEqual(bytes, nls);
            var crsEq = AdvSimd.CompareEqual(bytes, crs);
            var qtsEq = AdvSimd.CompareEqual(bytes, qts);
            var spsEq = AdvSimd.CompareEqual(bytes, sps);

            var lineEndings = AdvSimd.Or(nlsEq, crsEq);
            var lineEndingsSeparators = AdvSimd.Or(spsEq, lineEndings);
            var specialChars = AdvSimd.Or(lineEndingsSeparators, qtsEq);

            // Optimize for the case of no special character
            var specialCharMask = MoveMask(specialChars);
            if (specialCharMask != 0u)
            {
                var separatorsMask = MoveMask(spsEq);
                // Optimize for case of only separators i.e. no endings or quotes.
                // Add quote count to mask as hack to skip if quoting.
                var testMask = specialCharMask + quoteCount;
                if (separatorsMask == testMask)
                {
                    colInfosRefCurrent = ref ParseSeparatorsMask<TColInfo, TColInfoMethods>(
                        separatorsMask, charsIndex, ref colInfosRefCurrent);
                }
                else
                {
                    var separatorLineEndingsMask = MoveMask(lineEndingsSeparators);
                    if (separatorLineEndingsMask == testMask)
                    {
                        colInfosRefCurrent = ref ParseSeparatorsLineEndingsMasks<TColInfo, TColInfoMethods>(
                            separatorsMask, separatorLineEndingsMask,
                            ref charsRef, ref charsIndex, separator,
                            ref colInfosRefCurrent, ref lineNumber);
                        goto NEWROW;
                    }
                    else
                    {
                        var rowLineEndingOffset = 0;
                        colInfosRefCurrent = ref ParseAnyCharsMask<TColInfo, TColInfoMethods>(specialCharMask,
                            separator, ref charsRef, charsIndex,
                            ref rowLineEndingOffset, ref quoteCount,
                            ref colInfosRefCurrent, ref lineNumber);
                        // Used both to indicate row ended and if need to step +2 due to '\r\n'
                        if (rowLineEndingOffset != 0)
                        {
                            // Must be a col end and last is then dataIndex
                            charsIndex = TColInfoMethods.GetColEnd(colInfosRefCurrent) + rowLineEndingOffset;
                            goto NEWROW;
                        }
                    }
                }
            }
            goto LOOPSTEP;
        NEWROW:
            var colCount = TColInfoMethods.CountOffset(ref colInfosRef, ref colInfosRefCurrent);
            // Add new parsed row
            ref var parsedRowRef = ref MemoryMarshal.GetArrayDataReference(s._parsedRows);
            Add(ref parsedRowRef, s._parsedRowsCount) = new(lineNumber, colCount);
            ++s._parsedRowsCount;
            // Next row start (one before)
            colInfosRefCurrent = ref Add(ref colInfosRefCurrent, 1);
            A.Assert(IsAddressLessThan(ref colInfosRefCurrent, ref colInfosRefEnd));
            colInfosRefCurrent = TColInfoMethods.Create(charsIndex - 1, 0);
            // Update for next row
            colInfosRef = ref colInfosRefCurrent;
            s._parsingRowColEndsOrInfosStartIndex += colCount + 1;
            s._parsingRowCharsStartIndex = charsIndex;
            // Space for more rows?
            if (s._parsedRowsCount < s._parsedRows.Length)
            {
                goto LOOPNOSTEP;
            }
        }
        // Update instance state from enregistered
        _quoteCount = quoteCount;
        s._parsingRowColCount = TColInfoMethods.CountOffset(ref colInfosRef, ref colInfosRefCurrent);
        s._parsingLineNumber = lineNumber;
        // Step is VecUI8.Count so may go past end, ensure limited
        s._charsParseStart = Math.Min(charsEnd, charsIndex);
    }

    // MoveMask remains the same, using the cross-platform intrinsic
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    //static nuint MoveMask(VecUI8 v) => AdvSimd.Arm64.IsSupported ? MoveMaskAddAcross(v) : v.ExtractMostSignificantBits();
    static nuint MoveMask(VecUI8 v) => v.ExtractMostSignificantBits();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static byte PopCount(VecUI8 v) => AdvSimd.Arm64.AddAcross(Vec.ShiftRightLogical(v, 7)).ToScalar();

    // Assumes all bits set, not just MSB
    // https://community.arm.com/arm-community-blogs/b/servers-and-cloud-computing-blog/posts/porting-x86-vector-bitmask-optimizations-to-arm-neon
    // https://branchfree.org/2019/04/01/fitting-my-head-through-the-arm-holes-or-two-sequences-to-substitute-for-the-missing-pmovmskb-instruction-on-arm-neon/
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static ushort MoveMaskAddAcross(VecUI8 v)
    {
        //var s = AdvSimd.ShiftRightLogicalNarrowingLower(v, 4);
        var input = v.AsUInt16();
        // TODO: Move bitmask to register
        var bitmask = Vec.Create(0x0101, 0x0202, 0x0404, 0x0808, 0x1010, 0x2020, 0x4040, 0x8080);
        //var bitmask = Vec.Create<ushort>(0x0102, 0x0408, 0x1020, 0x4080, 0x0102, 0x0408, 0x1020, 0x4080);
        var and = AdvSimd.And(input, bitmask);
        return AdvSimd.Arm64.AddAcross(and).ToScalar();
    }

    //uint16_t neonmovemask_addv(uint8x16_t input8)
    //{
    //    uint16x8_t input = vreinterpretq_u16_u8(input8);
    //    const uint16x8_t bitmask = { 0x0101, 0x0202, 0x0404, 0x0808, 0x1010, 0x2020, 0x4040, 0x8080 };
    //    uint16x8_t minput = vandq_u16(input, bitmask);
    //    return vaddvq_u16(minput);
    //}

    //uint16_t neonmovemask_addv(uint8x16_t input8)
    //{
    //    uint16x8_t input = vreinterpretq_u16_u8(input8);
    //    const uint16x8_t bitmask = { 0x0101, 0x0202, 0x0404, 0x0808, 0x1010, 0x2020, 0x4040, 0x8080 };
    //    uint16x8_t minput = vandq_u16(input, bitmask);
    //    return vaddvq_u16(minput);
    //}
}
