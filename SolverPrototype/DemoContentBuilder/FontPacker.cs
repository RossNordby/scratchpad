using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DemoContentLoader;
using System.Diagnostics;

namespace DemoContentBuilder
{
    public class FontPacker
    {
        int atlasWidth;
        int alignmentMask;
        int padding;
        int paddingx2;

        int start;
        int rowIndex;
        public int Height { get; private set; }

        struct Interval
        {
            public int Start;
            public int End; //Technically redundant, but it simplifies the implementation a little. Performance doesn't matter.
            public int Height;
        }
        List<Interval> intervals;

        public FontPacker(int width, int mipLevels, int padding, int characterCount)
        {
            this.atlasWidth = width;
            this.alignmentMask = (1 << mipLevels) - 1;
            this.padding = padding;
            this.paddingx2 = padding * 2;

            intervals = new List<Interval>(characterCount);
            intervals.Add(new Interval { Start = 0, End = atlasWidth, Height = 0 });
        }


        int AddAndGetBaseHeight(int queryStart, int queryEnd, int newGlyphHeight)
        {
            Debug.Assert(queryStart >= 0 && queryStart < atlasWidth && queryEnd > 0 && queryEnd <= atlasWidth);
            //Performance doesn't matter here. Just scan through the interval set until the first interval that overlaps the query interval.
            int startIntervalIndex = -1;
            int baseHeight = 0;
            for (int i = 0; i < intervals.Count; ++i)
            {
                var interval = intervals[i];
                if (interval.Start < queryEnd && interval.End > queryStart)
                {
                    baseHeight = interval.Height;
                    startIntervalIndex = i;
                    break;
                }
            }
            Debug.Assert(startIntervalIndex >= 0);
            //Continue until we find the first interval that doesn't overlap the query interval.
            int endIntervalIndex = intervals.Count; //Exclusive bound.
            for (int i = startIntervalIndex + 1; i < intervals.Count; ++i)
            {
                var interval = intervals[i];
                if (interval.Start >= queryEnd || interval.End <= queryStart)
                {
                    endIntervalIndex = i;
                    break;
                }
                if (interval.Height > baseHeight)
                    baseHeight = interval.Height;
            }

            //Align and round up base height.
            baseHeight = (baseHeight + alignmentMask) & (~alignmentMask);

            //We know that the new glyph will be the highest point, so we can replace the interval section with it.
            //Change the endpoint of the first overlapped interval to be the beginning of the new interval.
            //Note that the constructor guarantees that there will always be at least one overlapped interval.
            var modifiedStartInterval = intervals[startIntervalIndex];
            modifiedStartInterval.End = queryStart;
            intervals[startIntervalIndex] = modifiedStartInterval;
            var overlappedIntervalCount = endIntervalIndex - startIntervalIndex;
            if (overlappedIntervalCount > 1)
            {
                var modifiedEndInterval = intervals[endIntervalIndex - 1];
                modifiedEndInterval.Start = queryEnd;
                intervals[endIntervalIndex - 1] = modifiedEndInterval;
            }
            var newInterval = new Interval { Start = queryStart, End = queryEnd, Height = newGlyphHeight + baseHeight };
            if (overlappedIntervalCount > 2)
            {
                //Remove all intervals which are fully contained within the new character's interval, indices [startIntervalIndex + 1, endIntervalIndex - 2].
                //Leave one in place so that we can replace it without another memory-bumping insertion.
                intervals.RemoveRange(startIntervalIndex + 1, overlappedIntervalCount - 2);
                intervals[startIntervalIndex + 1] = newInterval;
            }
            else
            {
                //The overlapped interval count is 2 or 1. We've already adjusted the first overlapped interval and, if it exists, the last overlapped interval.
                //All that's left is to insert a new interval for the new object.
                intervals.Insert(startIntervalIndex + 1, newInterval);
            }
            return baseHeight;
        }

        private void FillCharacterMinimumAndMove(ref CharacterData characterData, int end)
        {
            characterData.SourceMinimum.X = padding + start;
            characterData.SourceMinimum.Y = padding + AddAndGetBaseHeight(start, end, (int)characterData.SourceSpan.Y + paddingx2);
            start = end;
        }
        

        public void Add(ref CharacterData characterData)
        {
            int allocationWidth = (int)(paddingx2 + characterData.SourceSpan.X);
            if (allocationWidth > atlasWidth)
            {
                throw new ArgumentException(
                    "A single character that's wider than the entire atlas isn't gonna work. Is the FontPacker incorrectly initialized? Is the rasterized font size ridiculously huge?");
            }
            if ((rowIndex & 1) == 0)
            {
                //Place glyphs from left to right.
                start = (start + alignmentMask) & (~alignmentMask);
                var end = start + allocationWidth;

                if (end <= atlasWidth)
                {
                    FillCharacterMinimumAndMove(ref characterData, end);
                }
                else
                {
                    //This glyph doesn't fit; need to move to the next row.
                    ++rowIndex;
                    start = atlasWidth;
                    Add(ref characterData);
                }
            }
            else
            {
                //Place glyphs from right to left.
                start -= allocationWidth;
                if (start >= 0)
                {
                    //Delayed alignment; alignment will never make this negative.
                    start = start & (~alignmentMask);
                    var end = start + allocationWidth;
                    FillCharacterMinimumAndMove(ref characterData, end);
                }
                else
                {
                    //This glyph doesn't fit; need to move to the next row.
                    ++rowIndex;
                    start = 0;
                    Add(ref characterData);
                }

            }
        }

    }
}
