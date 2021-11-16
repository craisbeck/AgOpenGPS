﻿using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CContour
    {
        //copy of the mainform address
        private readonly FormGPS mf;

        public bool isContourOn, isContourBtnOn, isLocked = false;

        private int stripNum, lastLockPt = int.MaxValue, backSpacing = 30;
        public int ContourIndex = -1;

        //list of points for the new contour line
        public List<vec3> curList = new List<vec3>(128); 

        //constructor
        public CContour(FormGPS _f)
        {
            mf = _f;
        }

        public void BuildCurrentContourList(vec3 pivot)
        {
            mf.gyd.lastSecond = mf.secondsSinceStart;
            int ptCount;
            double minDistA = double.MaxValue;
            int start, stop;

            int pt = 0;

            if (stripNum < 0 || !isLocked)
            {
                stripNum = -1;
                for (int s = 0; s < mf.gyd.refList.Count; s++)
                {
                    if (mf.gyd.refList[s].Mode == Mode.BoundaryContour || mf.gyd.refList[s].Mode == Mode.Contour)
                    {
                        //if making a new strip ignore the last part or it will win always
                        if (s == ContourIndex)
                            ptCount = mf.gyd.refList[s].curvePts.Count - (int)Math.Max(backSpacing, mf.tool.toolWidth);
                        else
                            ptCount = mf.gyd.refList[s].curvePts.Count;

                        if (ptCount < 2) continue;
                        double dist;
                        bool last = true;

                        for (int p = 0; p < ptCount || last; p += 6)
                        {
                            if (p >= ptCount)
                            {
                                last = false;
                                p = ptCount - 1;
                            }
                            dist = ((pivot.easting - mf.gyd.refList[s].curvePts[p].easting) * (pivot.easting - mf.gyd.refList[s].curvePts[p].easting))
                                + ((pivot.northing - mf.gyd.refList[s].curvePts[p].northing) * (pivot.northing - mf.gyd.refList[s].curvePts[p].northing));
                            if (dist < minDistA)
                            {
                                minDistA = dist;
                                stripNum = s;
                                lastLockPt = p;
                            }
                        }
                    }
                }
            }

            int currentStripBox = stripNum == ContourIndex ? (int)Math.Max(backSpacing, mf.tool.toolWidth) : 1;

            if (stripNum < 0 || (ptCount = mf.gyd.refList[stripNum].curvePts.Count - currentStripBox) < 2)
            {
                curList.Clear();
                isLocked = false;
                return;
            }

            start = lastLockPt - 10; if (start < 0) start = 0;
            stop = lastLockPt + 10; if (stop > ptCount) stop = ptCount;

            //determine closest point
            double minDistance = double.MaxValue;

            for (int i = start; i < stop; i++)
            {
                double dist = ((pivot.easting - mf.gyd.refList[stripNum].curvePts[i].easting) * (pivot.easting - mf.gyd.refList[stripNum].curvePts[i].easting))
                    + ((pivot.northing - mf.gyd.refList[stripNum].curvePts[i].northing) * (pivot.northing - mf.gyd.refList[stripNum].curvePts[i].northing));

                if (dist < minDistance)
                {
                    minDistance = dist;
                    pt = lastLockPt = i;
                }
            }

            minDistance = Math.Sqrt(minDistance);

            if (minDistance > (isLocked ? 2.0 : 2.6) * mf.tool.toolWidth)
            {
                curList.Clear();
                isLocked = false;
                return;
            }

            //now we have closest point, the distance squared from it, and which patch and point its from
            
            double dy = mf.gyd.refList[stripNum].curvePts[pt + 1].easting - mf.gyd.refList[stripNum].curvePts[pt].easting;
            double dx = mf.gyd.refList[stripNum].curvePts[pt + 1].northing - mf.gyd.refList[stripNum].curvePts[pt].northing;

            //how far are we away from the reference line at 90 degrees - 2D cross product and distance
            double distanceFromRefLine = ((dx * pivot.easting) - (dy * pivot.northing) + (mf.gyd.refList[stripNum].curvePts[pt + 1].easting
                                    * mf.gyd.refList[stripNum].curvePts[pt].northing) - (mf.gyd.refList[stripNum].curvePts[pt + 1].northing * mf.gyd.refList[stripNum].curvePts[pt].easting))
                                    / Math.Sqrt((dx * dx) + (dy * dy));

            double heading = Math.Atan2(dy, dx);
            //are we going same direction as stripList was created?
            if(!isLocked)
                mf.gyd.isHeadingSameWay = Math.PI - Math.Abs(Math.Abs(mf.fixHeading - mf.gyd.refList[stripNum].curvePts[pt].heading) - Math.PI) < 1.57;

            double RefDist = (distanceFromRefLine + (mf.gyd.isHeadingSameWay ? mf.tool.toolOffset : -mf.tool.toolOffset)) / (mf.tool.toolWidth - mf.tool.toolOverlap);

            double howManyPathsAway;

            if (RefDist < 0) howManyPathsAway = (int)(RefDist - 0.5);
            else howManyPathsAway = (int)(RefDist + 0.5);

            if (howManyPathsAway >= -2 && howManyPathsAway <= 2)
            {

                curList.Clear();

                //don't guide behind yourself
                if (stripNum == ContourIndex && howManyPathsAway == 0) return;

                //make the new guidance line list called guideList
                ptCount = mf.gyd.refList[stripNum].curvePts.Count;

                //shorter behind you
                if (mf.gyd.isHeadingSameWay)
                {
                    start = pt - 6; if (start < 0) start = 0;
                    stop = pt + 45; if (stop > ptCount) stop = ptCount;
                }
                else
                {
                    start = pt - 45; if (start < 0) start = 0;
                    stop = pt + 6; if (stop > ptCount) stop = ptCount;
                }

                //if (howManyPathsAway != 0 && (mf.tool.halfToolWidth < (0.5*mf.tool.toolOffset)))
                {
                    double distAway = (mf.tool.toolWidth - mf.tool.toolOverlap) * howManyPathsAway + (mf.gyd.isHeadingSameWay ? -mf.tool.toolOffset : mf.tool.toolOffset);
                    double distSqAway = (distAway * distAway) * 0.97;


                    for (int i = start; i < stop; i++)
                    {
                        vec3 point = new vec3(
                            mf.gyd.refList[stripNum].curvePts[i].easting + (Math.Cos(mf.gyd.refList[stripNum].curvePts[i].heading) * distAway),
                            mf.gyd.refList[stripNum].curvePts[i].northing - (Math.Sin(mf.gyd.refList[stripNum].curvePts[i].heading) * distAway),
                            mf.gyd.refList[stripNum].curvePts[i].heading);

                        bool Add = true;
                        //make sure its not closer then 1 eq width
                        for (int j = start; j < stop; j++)
                        {
                            double check = glm.DistanceSquared(point.northing, point.easting, mf.gyd.refList[stripNum].curvePts[j].northing, mf.gyd.refList[stripNum].curvePts[j].easting);
                            if (check < distSqAway)
                            {
                                //Add = false;
                                break;
                            }
                        }
                        if (Add)
                        {
                            double dist = curList.Count > 0 ? ((point.easting - curList[curList.Count - 1].easting) * (point.easting - curList[curList.Count - 1].easting))
                                + ((point.northing - curList[curList.Count - 1].northing) * (point.northing - curList[curList.Count - 1].northing)) : 2.0;
                            if (dist > 0.3)
                                curList.Add(point);
                        }
                    }
                }

                int ptc = curList.Count;
                if (ptc < 5)
                {
                    curList.Clear();
                    isLocked = false;
                    return;
                }
            }
            else
            {
                curList.Clear();
                isLocked = false;
                return;
            }
        }

        //Add current position to stripList
        public void AddPoint(vec3 pivot)
        {
            if (!isContourOn)
            {
                isContourOn = true;

                mf.gyd.refList.Add(new CGuidanceLine(Mode.Contour));
                ContourIndex = mf.gyd.refList.Count - 1;
            }

            if (ContourIndex > -1)
                mf.gyd.refList[ContourIndex].curvePts.Add(new vec3(pivot.easting + Math.Cos(pivot.heading) * mf.tool.toolOffset, pivot.northing - Math.Sin(pivot.heading) * mf.tool.toolOffset, pivot.heading));
        }

        //End the strip
        public void StopContourLine()
        {
            //make sure its long enough to bother
            if (ContourIndex > -1 && mf.gyd.refList[ContourIndex].curvePts.Count > 5)
            {
                //build tale
                double head = mf.gyd.refList[ContourIndex].curvePts[0].heading;
                int length = (int)mf.tool.toolWidth + 3;
                vec3 pnt;
                for (int a = 0; a < length; a++)
                {
                    pnt.easting = mf.gyd.refList[ContourIndex].curvePts[0].easting - (Math.Sin(head));
                    pnt.northing = mf.gyd.refList[ContourIndex].curvePts[0].northing - (Math.Cos(head));
                    pnt.heading = mf.gyd.refList[ContourIndex].curvePts[0].heading;
                    mf.gyd.refList[ContourIndex].curvePts.Insert(0, pnt);
                }

                int ptc = mf.gyd.refList[ContourIndex].curvePts.Count - 1;
                head = mf.gyd.refList[ContourIndex].curvePts[ptc].heading;

                for (double i = 1; i < length; i += 1)
                {
                    pnt.easting = mf.gyd.refList[ContourIndex].curvePts[ptc].easting + (Math.Sin(head) * i);
                    pnt.northing = mf.gyd.refList[ContourIndex].curvePts[ptc].northing + (Math.Cos(head) * i);
                    pnt.heading = head;
                    mf.gyd.refList[ContourIndex].curvePts.Add(pnt);
                }

                //add the point list to the save list for appending to contour file
                mf.contourSaveList.Add(mf.gyd.refList[ContourIndex].curvePts);
            }
            else if (ContourIndex > -1)
            {
                mf.gyd.refList.RemoveAt(ContourIndex);
                if (mf.curve.selectedCurveIndex >= ContourIndex) mf.curve.selectedCurveIndex--;
                if (mf.ABLine.selectedABIndex >= ContourIndex) mf.ABLine.selectedABIndex--;
            }

            ContourIndex = -1;
            //turn it off
            isContourOn = false;
        }

        //build contours for boundaries
        public void BuildFenceContours(int pass, int spacingInt)
        {
            if (mf.bnd.bndList.Count == 0)
            {
                mf.TimedMessageBox(1500, "Boundary Contour Error", "No Boundaries Made");
                return;
            }

            vec3 point = new vec3();
            double totalHeadWidth;
            int signPass;

            if (pass == 1)
            {
                signPass = -1;
                //determine how wide a headland space
                totalHeadWidth = ((mf.tool.toolWidth - mf.tool.toolOverlap) * 0.5) - spacingInt;
            }

            else
            {
                signPass = 1;
                totalHeadWidth = ((mf.tool.toolWidth - mf.tool.toolOverlap) * pass) + spacingInt +
                    ((mf.tool.toolWidth - mf.tool.toolOverlap) * 0.5);
            }

            //totalHeadWidth = (mf.tool.toolWidth - mf.tool.toolOverlap) * 0.5 + 0.2 + (mf.tool.toolWidth - mf.tool.toolOverlap);

            for (int j = 0; j < mf.bnd.bndList.Count; j++)
            {
                //count the points from the boundary
                int ptCount = mf.bnd.bndList[j].fenceLine.Points.Count;

                CGuidanceLine ptList = new CGuidanceLine(Mode.BoundaryContour);

                for (int i = 0; i < ptCount; i++)
                {
                    //calculate the point inside the boundary
                    point.easting = mf.bnd.bndList[j].fenceLine.Points[i].easting - (signPass * Math.Sin(glm.PIBy2 + mf.bnd.bndList[j].fenceLine.Points[i].heading) * -totalHeadWidth);
                    point.northing = mf.bnd.bndList[j].fenceLine.Points[i].northing - (signPass * Math.Cos(glm.PIBy2 + mf.bnd.bndList[j].fenceLine.Points[i].heading) * -totalHeadWidth);

                    point.heading = mf.bnd.bndList[j].fenceLine.Points[i].heading - (j == 0 ? Math.PI : 0);
                    if (point.heading < -glm.twoPI) point.heading += glm.twoPI;

                    //only add if inside actual field boundary
                    ptList.curvePts.Add(point);
                }
                mf.gyd.refList.Add(ptList);
            }

            mf.TimedMessageBox(1500, "Boundary Contour", "Contour Path Created");
        }

        //draw the red follow me line
        public void DrawContourLine()
        {
            ////draw the guidance line
            int ptCount = curList.Count;
            if (ptCount < 2) return;
            GL.LineWidth(mf.ABLine.lineWidth);
            GL.Color3(0.98f, 0.2f, 0.980f);
            GL.Begin(PrimitiveType.LineStrip);
            for (int h = 0; h < ptCount; h++) GL.Vertex3(curList[h].easting, curList[h].northing, 0);
            GL.End();

            GL.PointSize(mf.ABLine.lineWidth);
            GL.Begin(PrimitiveType.Points);

            GL.Color3(0.87f, 08.7f, 0.25f);
            for (int h = 0; h < ptCount; h++) GL.Vertex3(curList[h].easting, curList[h].northing, 0);

            GL.End();

            //Draw the captured ref strip, red if locked
            if (isLocked)
            {
                GL.Color3(0.983f, 0.2f, 0.20f);
                GL.LineWidth(4);
            }
            else
            {
                GL.Color3(0.3f, 0.982f, 0.0f);
                GL.LineWidth(mf.ABLine.lineWidth);
            }

            //GL.PointSize(6.0f);
            GL.Begin(PrimitiveType.Points);
            for (int h = 0; h < mf.gyd.refList[stripNum].curvePts.Count; h++) GL.Vertex3(mf.gyd.refList[stripNum].curvePts[h].easting, mf.gyd.refList[stripNum].curvePts[h].northing, 0);
            GL.End();

            //GL.Begin(PrimitiveType.Points);
            //GL.Color3(1.0f, 0.95f, 0.095f);
            //GL.Vertex3(rEastCT, rNorthCT, 0.0);
            //GL.End();
            //GL.PointSize(1.0f);

            //GL.Color3(0.98f, 0.98f, 0.50f);
            //GL.Begin(PrimitiveType.LineStrip);
            //GL.Vertex3(boxE.easting, boxE.northing, 0);
            //GL.Vertex3(boxA.easting, boxA.northing, 0);
            //GL.Vertex3(boxD.easting, boxD.northing, 0);
            //GL.Vertex3(boxG.easting, boxG.northing, 0);
            //GL.Vertex3(boxE.easting, boxE.northing, 0);
            //GL.End();

            //GL.Begin(PrimitiveType.LineStrip);
            //GL.Vertex3(boxF.easting, boxF.northing, 0);
            //GL.Vertex3(boxH.easting, boxH.northing, 0);
            //GL.Vertex3(boxC.easting, boxC.northing, 0);
            //GL.Vertex3(boxB.easting, boxB.northing, 0);
            //GL.Vertex3(boxF.easting, boxF.northing, 0);
            //GL.End();

            ////draw the reference line
            //GL.PointSize(3.0f);
            ////if (isContourBtnOn)
            //{
            //    ptCount = stripList.Count;
            //    if (ptCount > 0)
            //    {
            //        ptCount = stripList[closestRefPatch].Count;
            //        GL.Begin(PrimitiveType.Points);
            //        for (int i = 0; i < ptCount; i++)
            //        {
            //            GL.Vertex2(stripList[closestRefPatch][i].easting, stripList[closestRefPatch][i].northing);
            //        }
            //        GL.End();
            //    }
            //}

            //ptCount = conList.Count;
            //if (ptCount > 0)
            //{
            //    //draw closest point and side of line points
            //    GL.Color3(0.5f, 0.900f, 0.90f);
            //    GL.PointSize(4.0f);
            //    GL.Begin(PrimitiveType.Points);
            //    for (int i = 0; i < ptCount; i++) GL.Vertex3(conList[i].x, conList[i].z, 0);
            //    GL.End();

            //    GL.Color3(0.35f, 0.30f, 0.90f);
            //    GL.PointSize(6.0f);
            //    GL.Begin(PrimitiveType.Points);
            //    GL.Vertex3(conList[closestRefPoint].x, conList[closestRefPoint].z, 0);
            //    GL.End();
            //}

            if (mf.isPureDisplayOn && mf.gyd.distanceFromCurrentLinePivot != 32000 && !mf.isStanleyUsed)
            {
                //if (ppRadiusCT < 50 && ppRadiusCT > -50)
                //{
                //    const int numSegments = 100;
                //    double theta = glm.twoPI / numSegments;
                //    double c = Math.Cos(theta);//precalculate the sine and cosine
                //    double s = Math.Sin(theta);
                //    double x = ppRadiusCT;//we start at angle = 0
                //    double y = 0;

                //    GL.LineWidth(1);
                //    GL.Color3(0.795f, 0.230f, 0.7950f);
                //    GL.Begin(PrimitiveType.LineLoop);
                //    for (int ii = 0; ii < numSegments; ii++)
                //    {
                //        //glVertex2f(x + cx, y + cy);//output vertex
                //        GL.Vertex3(x + radiusPointCT.easting, y + radiusPointCT.northing, 0);//output vertex

                //        //apply the rotation matrix
                //        double t = x;
                //        x = (c * x) - (s * y);
                //        y = (s * t) + (c * y);
                //    }
                //    GL.End();
                //}

                //Draw lookahead Point
                GL.PointSize(6.0f);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(1.0f, 0.95f, 0.095f);
                GL.Vertex3(mf.gyd.goalPoint.easting, mf.gyd.goalPoint.northing, 0.0);
                GL.End();
                GL.PointSize(1.0f);
            }
        }

        //Reset the contour to zip
        public void ResetContour()
        {
            ContourIndex = -1;
            isContourOn = false;
            for (int i = mf.gyd.refList.Count - 1; i >= 0; i--)
            {
                if (mf.gyd.refList[i].Mode == Mode.BoundaryContour || mf.gyd.refList[i].Mode == Mode.Contour)
                {
                    mf.gyd.refList.RemoveAt(i);
                    if (mf.curve.selectedCurveIndex >= i) mf.curve.selectedCurveIndex--;
                    if (mf.ABLine.selectedABIndex >= i) mf.ABLine.selectedABIndex--;
                }
            }

            curList.Clear();
        }
    }//class
}//namespace