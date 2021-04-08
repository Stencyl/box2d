﻿/*
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

package box2D.common.math;
	
	
/**
* A 2-by-2 matrix. Stored in column-major order.
*/
class B2Mat22
{
	public function new ()
	{
		col1 = new B2Vec2(0, 1.0);
		col2 = new B2Vec2(0, 1.0);
	}
	
	public static function fromAngle(angle:Float):B2Mat22
	{
		var mat:B2Mat22 = new B2Mat22();
		mat.set(angle);
		return mat;
	}
	
	public static function fromVV(c1:B2Vec2, c2:B2Vec2):B2Mat22
	{
		var mat:B2Mat22 = new B2Mat22();
		mat.setVV(c1, c2);
		return mat;
	}

	public function set(angle:Float) : Void
	{
		var c:Float = Math.cos(angle);
		var s:Float = Math.sin(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}
	
	public function setVV(c1:B2Vec2, c2:B2Vec2) : Void
	{
		col1.setV(c1);
		col2.setV(c2);
	}
	
	public function copy():B2Mat22{
		var mat:B2Mat22 = new B2Mat22();
		mat.setM(this);
		return mat;
	}
	
	public function setM(m:B2Mat22) : Void
	{
		col1.setV(m.col1);
		col2.setV(m.col2);
	}
	
	public function addM(m:B2Mat22) : Void
	{
		col1.x += m.col1.x;
		col1.y += m.col1.y;
		col2.x += m.col2.x;
		col2.y += m.col2.y;
	}
	
	public function multiplyV(v:B2Vec2):B2Vec2
	{
		return new B2Vec2
		(
			col1.x * v.x + col2.x * v.y, 
			col1.y * v.x + col2.y * v.y
		);
	}

	public function multiplyVInPlace(v:B2Vec2)
	{
		v.set
		(
			col1.x * v.x + col2.x * v.y, 
			col1.y * v.x + col2.y * v.y
		);
	}	
	
	public function setIdentity() : Void
	{
		col1.x = 1.0; col2.x = 0.0;
		col1.y = 0.0; col2.y = 1.0;
	}

	public function setZero() : Void
	{
		col1.x = 0.0; col2.x = 0.0;
		col1.y = 0.0; col2.y = 0.0;
	}
	
	public function setAngle(n:Float) : Void 
	{
		col1.x = Math.cos(n);
		col1.y = Math.sin(n);
		col2.x = -col1.y;
		col2.y = col1.x;
	}
	
	public function getAngle() :Float
	{
		return Math.atan2(col1.y, col1.x);
	}

	/**
	 * Compute the inverse of this matrix, such that inv(A) * A = identity.
	 */
	public function getInverse(out:B2Mat22):B2Mat22
	{
		var a:Float = col1.x; 
		var b:Float = col2.x; 
		var c:Float = col1.y; 
		var d:Float = col2.y;
		//var B:B2Mat22 = new B2Mat22();
		var det:Float = a * d - b * c;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		out.col1.x =  det * d;	out.col2.x = -det * b;
		out.col1.y = -det * c;	out.col2.y =  det * a;
		return out;
	}
	
	// Solve A * x = b
	public function solve(out:B2Vec2, bX:Float, bY:Float):B2Vec2
	{
		//float32 a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		var a11:Float = col1.x;
		var a12:Float = col2.x;
		var a21:Float = col1.y;
		var a22:Float = col2.y;
		//float32 det = a11 * a22 - a12 * a21;
		var det:Float = a11 * a22 - a12 * a21;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		out.x = det * (a22 * bX - a12 * bY);
		out.y = det * (a11 * bY - a21 * bX);
		
		return out;
	}
	
	public function abs() : Void
	{
		col1.abs();
		col2.abs();
	}

	public var col1:B2Vec2;
	public var col2:B2Vec2;
}