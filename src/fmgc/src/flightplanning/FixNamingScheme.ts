
/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/** Generates fix names based on the ARINC default naming scheme. */
export class FixNamingScheme {
  private static alphabet: String[] = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'];

  /**
   * Generates a fix name for a course to distance type fix.
   * @param course The course that will be flown.
   * @param distance The distance along the course or from the reference fix.
   * @returns The generated fix name.
   */
  public static courseToDistance(course: number, distance: number): string {
    const roundedDistance = Math.round(distance);
    const distanceAlpha = distance > 26 ? 'Z' : this.alphabet[roundedDistance];

    return `D${course.toFixed(0).padStart(3, '0')}${distanceAlpha}`;
  }

  /**
   * Generates a fix name for a course turn to intercept type fix.
   * @param course The course that will be turned to.
   * @returns The generated fix name.
   */
  public static courseToIntercept(course: number): string {
    return `I${course.toFixed(0).padStart(3, '0')}`;
  }
}