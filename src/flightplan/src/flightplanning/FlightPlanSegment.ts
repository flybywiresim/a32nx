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
/**
 * A segment of a flight plan.
 */
export class FlightPlanSegment {

  /**
   * Creates a new FlightPlanSegment.
   * @param type The type of the flight plan segment.
   * @param offset The offset within the original flight plan that
   * the segment starts at.
   * @param waypoints The waypoints in the flight plan segment.
   */
  constructor(public type: SegmentType, public offset: number, public waypoints: WayPoint[]) {
  }

  /** An empty flight plan segment. */
  public static Empty: FlightPlanSegment = new FlightPlanSegment(-1, -1, []);
}

/** Types of flight plan segments. */
export enum SegmentType {

  /** The origin airfield segment. */
  Origin,

  /** The departure segment. */
  Departure,

  /** The enroute segment. */
  Enroute,

  /** The arrival segment. */
  Arrival,

  /** The approach segment. */
  Approach,

  /** The missed approach segment. */
  Missed,

  /** The destination airfield segment. */
  Destination
}