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

import { FlightPlanSegment } from "../wtsdk";

/**
 * Information about the current direct-to procedures in the flight plan.
 */
export class DirectTo {

  /** Whether or not the current direct-to is in the flight plan. */
  public waypointIsInFlightPlan: boolean = false;

  /** Whether or not direct-to is active. */
  public isActive: boolean = false;

  /** The current direct-to waypoint, if not part of the flight plan. */
  public waypoint?: WayPoint;

  /** The current direct-to waypoint index, if part of the flight plan. */
  public planWaypointIndex = 0;

  /** The intercept points towards the direct. */
  public interceptPoints?: WayPoint[];

  /** The current active index in the direct to waypoints. */
  public currentWaypointIndex = 0;

  /** The segments of the direct plan. */
  public segments?: FlightPlanSegment[];
}