/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package box2D.dynamics.contacts;

import box2D.collision.B2Manifold;
import box2D.collision.B2ManifoldPoint;
import box2D.collision.shapes.B2CircleShape;
import box2D.collision.shapes.B2EdgeShape;
import box2D.common.math.B2Transform;
import box2D.common.math.B2Vec2;
import box2D.common.math.B2Mat22;
import box2D.common.math.B2Math;
import box2D.dynamics.B2Body;
import box2D.dynamics.B2Fixture;
import box2D.dynamics.contacts.B2Contact;

class B2EdgeAndCircleContact extends B2Contact
{
	static public function create(allocator:Dynamic):B2Contact
	{
		return new B2EdgeAndCircleContact();
	}

	static public function destroy(contact:B2Contact, allocator:Dynamic):Void
	{
	}

	public override function reset(fixtureA:B2Fixture = null, fixtureB:B2Fixture = null):Void
	{
		super.reset(fixtureA, fixtureB);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_circleShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
	}

	//~b2EdgeAndCircleContact() {}

	public override function evaluate():Void
	{
		var bA:B2Body = m_fixtureA.getBody();
		var bB:B2Body = m_fixtureB.getBody();

		b2CollideEdgeAndCircle
		(
			m_manifold,
			cast(m_fixtureA.getShape(), B2EdgeShape), bA.m_xf,
			cast(m_fixtureB.getShape(), B2CircleShape), bB.m_xf
		);
	}

	private function b2CollideEdgeAndCircle(manifold:B2Manifold,
			edge:B2EdgeShape, 
			xf1:B2Transform,
			circle:B2CircleShape, 
			xf2:B2Transform):Void
	{
		manifold.m_pointCount = 0;

		var tPoint:B2ManifoldPoint;
		var dX:Float = 0;
		var dY:Float = 0;
		var positionX:Float = 0;
		var positionY:Float = 0;
		var tVec:B2Vec2;
		var tMat:B2Mat22;
		
		tMat = xf2.R;
		tVec = circle.m_p;
		
		var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		
		var cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
		var cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);
		var dist:Float = 0;
		var radius = edge.m_radius + circle.m_radius;
		tVec = edge.m_normal;
		var separation = tVec.x * dX + tVec.y * dY;
		var v1 = edge.m_v1;
		var v2 = edge.m_v2;
		
		if(separation < B2Math.MIN_VALUE) 
		{
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.setV(edge.m_normal);
			manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
			manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
			manifold.m_points[0].m_localPoint.setV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
			return;
		}
		
		var u1:Float = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
		var u2:Float = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
		
		if(u1 <= 0.0) 
		{
			if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) 
				return;
			
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v1.x;
			manifold.m_localPlaneNormal.y = cLocalY - v1.y;
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.setV(v1);
			manifold.m_points[0].m_localPoint.setV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		
		else if(u2 <= 0) 
		{
			if((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) 
				return;
			
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v2.x;
			manifold.m_localPlaneNormal.y = cLocalY - v2.y;
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.setV(v2);
			manifold.m_points[0].m_localPoint.setV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		
		else 
		{
			var faceCenterX:Float = 0.5 * (v1.x + v2.x);
			var faceCenterY:Float = 0.5 * (v1.y + v2.y);
			
			separation = (cLocalX - faceCenterX) * tVec.x + (cLocalY - faceCenterY) * tVec.y;
			
			if(separation > radius) 
				return;
			
			manifold.m_pointCount = 1;
			manifold.m_type = B2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = tVec.x;
			manifold.m_localPlaneNormal.y = tVec.y;
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(faceCenterX, faceCenterY);
			manifold.m_points[0].m_localPoint.setV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
	}
}
