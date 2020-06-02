#region Math Variables
#if UNIGINE_DOUBLE
using Scalar = System.Double;
using Vec2 = Unigine.dvec2;
using Vec3 = Unigine.dvec3;
using Vec4 = Unigine.dvec4;
using Mat4 = Unigine.dmat4;
#else
using Scalar = System.Single;
using Vec2 = Unigine.vec2;
using Vec3 = Unigine.vec3;
using Vec4 = Unigine.vec4;
using Mat4 = Unigine.mat4;
using WorldBoundBox = Unigine.BoundBox;
using WorldBoundSphere = Unigine.BoundSphere;
using WorldBoundFrustum = Unigine.BoundFrustum;
#endif
#endregion

using System.Collections.Generic;
using Unigine;

[Component(PropertyGuid = "df4f4519e86e6ba48bf1c2b3aec42876380a055d")]
public class FirstPersonController : Component
{
	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Move forward key")]
	private Input.KEY forwardKey = Input.KEY.W;

	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Move backward key")]
	private Input.KEY backwardKey = Input.KEY.S;

	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Move right key")]
	private Input.KEY rightKey = Input.KEY.D;

	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Move left key")]
	private Input.KEY leftKey = Input.KEY.A;

	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Run mode activation key")]
	private Input.KEY runKey = Input.KEY.SHIFT;

	[ShowInEditor]
	[Parameter(Group = "Input", Tooltip = "Jump key")]
	private Input.KEY jumpKey = Input.KEY.SPACE;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Input", Tooltip = "Mouse sensitivity multiplier")]
	private float mouseSensitivity = 1.0f;

	[ShowInEditor]
	[Parameter(Group = "Common", Tooltip = "You can use an ObjectDummy to customize the body.\nIt should have a dummy body and a capsule shape assigned.\n" +
		"Mass, collision height, collision radius, physical intersection\nmask and collision mask will be set automatically according to shape parameters.\n" +
		"In case of incorrect settings of the specified custom body, a default body shall be created instead.")]
	private bool useExternalBody = false;

	[ShowInEditor]
	[Parameter(Group = "Common", Tooltip = "ObjectDummy with a dummy body and a capsule shape")]
	[ParameterCondition(nameof(useExternalBody), 1)]
	private ObjectDummy playerBody = null;

	[ShowInEditor]
	[ParameterMask(Group = "Common", Tooltip = "Mask used for selective detections of physics intersections (between physical objects with bodies and collider shapes, or ray intersections with collider geometry)")] // TO DO: add physical intersection mask to MaskType
	[ParameterCondition(nameof(useExternalBody), 0)]
	private int physicalIntersectionMask = 1;

	[ShowInEditor]
	[ParameterMask(MaskType = ParameterMaskAttribute.TYPE.COLLISION, Group = "Common", Tooltip = "Mask used for selective collision detection")]
	[ParameterCondition(nameof(useExternalBody), 0)]
	private int collisionMask = 1;

	[ShowInEditor]
	[ParameterMask(MaskType = ParameterMaskAttribute.TYPE.INTERSECTION, Group = "Common", Tooltip = "Mask used for contacts correction")]
	private int contactsIntersectionMask = 1;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Physical mass of the player")]
	[ParameterCondition(nameof(useExternalBody), 0)]
	private float mass = 60.0f;

	[ShowInEditor]
	[Parameter(Group = "Common", Tooltip = "Toggles collision detection for the player on and off")]
	private bool isCollision = true;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Radius of the capsule shape")]
	[ParameterCondition(nameof(useExternalBody), 0)]
	private float collisionRadius = 0.3f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Height of the capsule shape (cylindrical part only)")]
	[ParameterCondition(nameof(useExternalBody), 0)]
	private float collisionHeight = 1.15f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Static friction of the player")]
	private float friction = 2.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Walking speed of the player")]
	private float walkSpeed = 4.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Running speed of the player")]
	private float runSpeed = 8.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Horizontal acceleration of the player")]
	private float acceleration = 25.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Horizontal damping of the player's speed")]
	private float damping = 25.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Jumping power of the player")]
	private float jumpPower = 6.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Common", Tooltip = "Gravity multiplier for the player")]
	private float gravityMultiplier = 1.5f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Camera", Tooltip = "Camera height above the player’s feet")]
	private float cameraHeight = 1.65f;

	[ShowInEditor]
	[ParameterSlider(Min = -89.9f, Max = 89.9f, Group = "Camera", Tooltip = "The minimum vertical angle of the camera (look down)")]
	private float minThetaAngle = -89.9f;

	[ShowInEditor]
	[ParameterSlider(Min = -89.9f, Max = 89.9f, Group = "Camera", Tooltip = "The maximum vertical angle of the camera (look up)")]
	private float maxThetaAngle = 89.9f;

	[ShowInEditor]
	[Parameter(Group = "Auto Stepping", Tooltip = "Toggles automatic walking up and down certain obstacles (stairs, for example)")]
	private bool enableAutoStepping = true;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Auto Stepping", Tooltip = "Minimum height for automatic walk up.\nEnables avoiding false positives on flat surfaces.")]
	[ParameterCondition(nameof(enableAutoStepping), 1)]
	private float minStepHeight = 0.05f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Auto Stepping", Tooltip = "Maximum height for automatic walk up")]
	[ParameterCondition(nameof(enableAutoStepping), 1)]
	private float maxStepHeight = 0.3f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Auto Stepping", Tooltip = "Maximum walkable slope angle.\n" +
		"If the slope angle of the surface under the player after making a step is greater than this value, the last automatic step is canceled.")]
	[ParameterCondition(nameof(enableAutoStepping), 1)]
	private float autoSteppingCancelAngle = 30.0f;

	[ShowInEditor]
	[Parameter(Group = "Slope Sliding", Tooltip = "Toggles additional sliding velocity on slopes on and off")]
	private bool enableSlopeSliding = true;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Max = 90.0f, Group = "Slope Sliding", Tooltip = "Maximum slope angle up to which additional sliding velocity is not applied")]
	[ParameterCondition(nameof(enableSlopeSliding), 1)]
	private float maxStaticSlopeAngle = 50.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Max = 90.0f, Group = "Slope Sliding", Tooltip = "Slope angle for which the maximum sliding velocity is applied")]
	[ParameterCondition(nameof(enableSlopeSliding), 1)]
	private float maxSlopeAngle = 65.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Slope Sliding", Tooltip = "Minimum sliding velocity to be applied when maxStaticSlopeAngle limit is exceeded")]
	[ParameterCondition(nameof(enableSlopeSliding), 1)]
	private float minSlidingSpeed = 1.0f;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Slope Sliding", Tooltip = "Maximum sliding velocity to be applied to be applied for the maxSlopeAngle")]
	[ParameterCondition(nameof(enableSlopeSliding), 1)]
	private float maxSlidingSpeed = 5.0f;

	[ShowInEditor]
	[Parameter(Group = "Objects Interaction", Tooltip = "Toggles physical interaction with rigid bodies on anf off")]
	private bool enableObjectsInteraction = true;

	[ShowInEditor]
	[ParameterSlider(Min = 0.0f, Group = "Objects Interaction", Tooltip = "Force multiplier to be applied to a rigid body colliding with the player")]
	[ParameterCondition(nameof(enableObjectsInteraction), 1)]
	private float forceMultiplier = 150.0f;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Toggles advanced player settings on and off")]
	private bool showAdvancedSettings = false;

	[ShowInEditor]
	[ParameterSlider(Min = 4, Max = 1500, Group = "Advanced Settings", Tooltip = "Maximum number of contacts to be processed for collisions")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private int contactsBufferSize = 50;

	[ShowInEditor]
	[ParameterSlider(Min = 15, Max = 240, Group = "Advanced Settings", Tooltip = "Minimum update rate for the player (in number of frames per second).\n" +
		"If this value exceeds the current framerate, the player will be updated several times per frame")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private int playerActorFps = 60;

	[ShowInEditor]
	[ParameterSlider(Min = 1, Group = "Advanced Settings", Tooltip = "Number of iterations to resolve collisions")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private int collisionIterations = 4;

	[ShowInEditor]
	[ParameterSlider(Min = 10, Max = 1500, Group = "Advanced Settings", Tooltip = "Maximum number of contacts up to which the number of iterations specified above shall be used.\n" +
		"After exceeding this value a single iteration shall be used to avoid performance degradation.")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private int heavyContactsCount = 100;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show debug information")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool showDebug = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show current player's basis")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderBasis = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show current velocity vector")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderVelocity = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show current camera direction vector")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderDirection = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show contacts on collisions when walking upstairs")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderUpPassContacts = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show contacts on sidestepping collisions")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderSidePassContacts = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show contacts on collisions when walking downstairs")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderDownPassContacts = false;

	[ShowInEditor]
	[Parameter(Group = "Advanced Settings", Tooltip = "Show current slope basis")]
	[ParameterCondition(nameof(showAdvancedSettings), 1)]
	private bool renderSlopeBasis = false;

	public int PhysicalIntersectionMask
	{
		get { return shape.PhysicsIntersectionMask; }
		set
		{
			physicalIntersectionMask = value;
			shape.PhysicsIntersectionMask = physicalIntersectionMask;
		}
	}

	public int CollisionMask
	{
		get { return shape.CollisionMask; }
		set
		{
			collisionMask = value;
			shape.CollisionMask = collisionMask;
		}
	}

	public float Mass
	{
		get { return shape.Mass; }
		set
		{
			mass = value;
			shape.Mass = mass;
		}
	}

	public bool IsCollision
	{
		get { return isCollision; }
		set { isCollision = value; }
	}

	public float CollisionRadius
	{
		get { return shape.Radius; }
		set
		{
			collisionRadius = MathLib.Max(0.0f, value);

			if (MathLib.Compare(shape.Radius, collisionRadius) < 1)
			{
				dummy.SetPreserveTransform(new Mat4(MathLib.Translate(vec3.UP * (collisionRadius - shape.Radius))) * dummy.Transform);
				shape.Radius = collisionRadius;
				WorldTransform = new Mat4(MathLib.Inverse(GetModelview()));
			}
		}
	}

	public float CollisionHeight
	{
		get { return shape.Height; }
		set
		{
			collisionHeight = MathLib.Max(0.0f, value);

			if (MathLib.Compare(shape.Height, collisionHeight) < 1)
			{
				dummy.SetPreserveTransform(new Mat4(MathLib.Translate(vec3.UP * (collisionHeight - shape.Height) * 0.5f)) * dummy.Transform);
				shape.Height = collisionHeight;
				WorldTransform = new Mat4(MathLib.Inverse(GetModelview()));
			}
		}
	}

	public float Friction
	{
		get { return friction; }
		set { friction = MathLib.Max(0.0f, value); }
	}

	public float WalkSpeed
	{
		get { return walkSpeed; }
		set { walkSpeed = MathLib.Max(0.0f, value); }
	}

	public float RunSpeed
	{
		get { return runSpeed; }
		set { runSpeed = MathLib.Max(0.0f, value); }
	}

	public float Acceleration
	{
		get { return acceleration; }
		set { acceleration = MathLib.Max(0.0f, value); }
	}

	public float Damping
	{
		get { return damping; }
		set { damping = MathLib.Max(0.0f, value); }
	}

	public float JumpPower
	{
		get { return jumpPower; }
		set { jumpPower = MathLib.Max(0.0f, value); }
	}

	public float GravityMultiplier
	{
		get { return gravityMultiplier; }
		set { gravityMultiplier = MathLib.Max(0.0f, value); }
	}

	public float MinStepHeight
	{
		get { return minStepHeight; }
		set { minStepHeight = MathLib.Max(0.0f, value); }
	}

	public float MaxStepHeight
	{
		get { return maxStepHeight; }
		set { maxStepHeight = MathLib.Max(0.0f, value); }
	}

	public float AutoSteppingCancelAngle
	{
		get { return autoSteppingCancelAngle; }
		set { autoSteppingCancelAngle = MathLib.Max(0.0f, value); }
	}

	public float CameraHeight
	{
		get { return cameraHeight; }
		set { cameraHeight = MathLib.Max(0.0f, value); }
	}

	public float MinThetaAngle
	{
		get { return minThetaAngle; }
		set { minThetaAngle = MathLib.Clamp(value, -89.9f, 89.9f); }
	}

	public float MaxThetaAngle
	{
		get { return maxThetaAngle; }
		set { maxThetaAngle = MathLib.Clamp(value, -89.9f, 89.9f); }
	}

	// don't use for internal setting
	public float ThetaAngle
	{
		get { return thetaAngle; }
		set
		{
			float angle = MathLib.Clamp(value, -MaxThetaAngle, -MinThetaAngle) - thetaAngle;
			direction = new quat(MathLib.Cross(vec3.UP, direction), angle) * direction;
			thetaAngle += angle;

			FlushTransform();
		}
	}

	// don't use for internal setting
	public float PhiAngle
	{
		get { return phiAngle; }
		set
		{
			float angle = value - phiAngle;
			direction = new quat(vec3.UP, angle) * direction;
			phiAngle += angle;

			FlushTransform();
		}
	}

	public bool IsControlled { get; private set; } = true;

	public bool IsGround { get; private set; } = false;

	public bool IsCeiling { get; private set; } = false;

	public bool IsFrozen { get; private set; } = false;

	public Mat4 Transform
	{
		get { return node.Transform; }
		set
		{
			node.Transform = value;
			UpdateTransform();
		}
	}

	public Mat4 WorldTransform
	{
		get { return node.WorldTransform; }
		set
		{
			node.WorldTransform = value;
			UpdateTransform();
		}
	}

	public vec3 ViewDirection
	{
		get { return direction; }
		set
		{
			direction = value.Normalized;

			// ortho basis
			vec3 tangent, binormal;
			Geometry.OrthoBasis(vec3.UP, out tangent, out binormal);

			// decompose direction
			// in this case don't use properties
			phiAngle = MathLib.Atan2(MathLib.Dot(direction, tangent), MathLib.Dot(direction, binormal)) * MathLib.RAD2DEG;
			thetaAngle = MathLib.Acos(MathLib.Clamp(MathLib.Dot(direction, vec3.UP), -1.0f, 1.0f)) * MathLib.RAD2DEG - 90.0f;

			FlushTransform();
		}
	}

	public int NumContacts => contacts.Count;

	public ShapeContact GetContact(int num)
	{
		return contacts[num];
	}

	private bool isInitialized = false;

	private BodyDummy dummy = null;
	private ShapeCapsule shape = null;

	private List<ShapeContact> contacts = null;

	private Camera camera = null;

	private vec3 velocity = vec3.ZERO;
	private vec3 moveImpulse = vec3.ZERO;
	private vec3 jumpImpulse = vec3.ZERO;
	private float targetSpeed = 0.0f;

	// current transform
	private Vec3 position = Vec3.ZERO;
	private vec3 direction = vec3.RIGHT;
	private float phiAngle = 0.0f;
	private float thetaAngle = 0.0f;

	// current basis
	private vec3 xAxis = vec3.RIGHT;
	private vec3 yAxis = vec3.FORWARD;
	private vec3 zAxis = vec3.UP;

	// autostepping
	private bool usedAutoStepping = false;
	private float lastStepHeight = 0.0f;
	private vec3 autoSteppingOffset = vec3.ZERO;

	// slope sliding
	private vec3 slopeNormal = vec3.UP;
	private Vec3 slopePoint = Vec3.ZERO;
	private float slopeAngle = 0.0f;

	private void Init()
	{
		if (showAdvancedSettings)
			showAdvancedSettings = false;

		if (useExternalBody)
			useExternalBody = false;

		PlayerDummy player = node as PlayerDummy;
		if (!player)
			return;

		// decompose transformation
		position = node.WorldPosition;
		direction = MathLib.Normalize(new vec3(-node.WorldTransform.AxisZ));

		camera = player.Camera;

		if (playerBody)
		{
			dummy = playerBody.Body as BodyDummy;
			if (dummy)
			{
				for (int i = 0; i < dummy.NumShapes; i++)
					if (!shape)
						shape = dummy.GetShape(i) as ShapeCapsule;

				if (shape)
				{
					shape.Restitution = 0.0f;
					shape.Continuous = false;

					PhysicalIntersectionMask = shape.PhysicsIntersectionMask;
					CollisionMask = shape.CollisionMask;
					Mass = shape.Mass;
					CollisionRadius = shape.Radius;
					CollisionHeight = shape.Height;
				}
			}
		}

		if (!dummy || !shape)
		{
			if (playerBody)
				playerBody.Enabled = false;

			playerBody = new ObjectDummy();
			dummy = new BodyDummy();

			shape = new ShapeCapsule(1.0f, 1.0f);
			shape.Restitution = 0.0f;
			shape.Continuous = false;

			dummy.Enabled = true;
			playerBody.Body = dummy;
			shape.Body = dummy;

			PhysicalIntersectionMask = physicalIntersectionMask;
			CollisionMask = collisionMask;
			Mass = mass;
			CollisionRadius = collisionRadius;
			CollisionHeight = collisionHeight;
		}

		contacts = new List<ShapeContact>();

		UpdateTransform();

		maxSlopeAngle = MathLib.Max(maxSlopeAngle, maxStaticSlopeAngle);
		maxSlidingSpeed = MathLib.Max(maxSlidingSpeed, minSlidingSpeed);

		if (showDebug)
		{
			Visualizer.Enabled = true;
			Render.ShowTriangles = 1;
		}

		isInitialized = true;
	}

	private void Update()
	{
		if (!isInitialized)
			return;

		float ifps = Game.IFps;

		UpdateMoveImpulse(ifps);

		float time = ifps * Physics.Scale;

		do
		{
			// adaptive time step
			float ifpsa = MathLib.Min(time, 1.0f / playerActorFps);
			time -= ifpsa;

			UpdateVelocity(ifpsa);
			UpdatePosition(ifpsa);
			UpdateCollisions(ifpsa);
		}
		while (time > MathLib.EPSILON);

		// current position
		playerBody.WorldTransform = GetBodyTransform();

		FlushTransform();

		if (renderBasis)
		{
			Visualizer.RenderVector(node.WorldPosition, node.WorldPosition + xAxis, new vec4(1, 0, 0, 1));
			Visualizer.RenderVector(node.WorldPosition, node.WorldPosition + yAxis, new vec4(0, 1, 0, 1));
			Visualizer.RenderVector(node.WorldPosition, node.WorldPosition + zAxis, new vec4(0, 0, 1, 1));
		}

		if (renderVelocity)
			Visualizer.RenderVector(node.WorldPosition, node.WorldPosition + velocity, new vec4(0, 0, 0, 1));

		if (renderDirection)
			Visualizer.RenderVector(camera.Position, camera.Position + direction, new vec4(1, 1, 0, 1));

		if (showDebug && shape)
			shape.RenderVisualizer(new vec4(0.0f, 0.0f, 1.0f, 1.0f));
	}

	private void UpdatePhysics()
	{
		if (!isInitialized)
			return;

		if (enableObjectsInteraction)
		{
			shape.GetCollision(contacts);
			foreach (var c in contacts)
				if (c.Object && c.Object.BodyRigid)
				{
					float speed = new vec3(MathLib.Dot(xAxis, velocity), MathLib.Dot(yAxis, velocity), MathLib.Dot(zAxis, velocity)).Length;
					speed = MathLib.Min(speed, 1.0f);
					c.Object.BodyRigid.AddWorldForce(c.Point, -c.Normal * speed * forceMultiplier);
				}
		}
	}

	private void Shutdown()
	{
		if (showDebug)
		{
			Visualizer.Enabled = false;
			Render.ShowTriangles = 0;
		}
	}

	private void UpdateMoveImpulse(float ifps)
	{
		// impulse
		moveImpulse = vec3.ZERO;

		if (!App.MouseGrab)
			return;

		// ortho basis
		vec3 tangent, binormal;
		Geometry.OrthoBasis(vec3.UP, out tangent, out binormal);

		// current basis
		xAxis = new quat(vec3.UP, -PhiAngle) * binormal;
		yAxis = MathLib.Normalize(MathLib.Cross(vec3.UP, xAxis));
		zAxis = MathLib.Normalize(MathLib.Cross(xAxis, yAxis));

		// controls
		if (IsControlled)
		{
			// old velocity
			float xSpeed = MathLib.Dot(xAxis, velocity);
			float ySpeed = MathLib.Dot(yAxis, velocity);
			float zSpeed = MathLib.Dot(zAxis, velocity);

			// direction
			// in this case don't use properties
			phiAngle += Input.MouseDelta.x * mouseSensitivity;
			thetaAngle += Input.MouseDelta.y * mouseSensitivity;
			thetaAngle = MathLib.Clamp(thetaAngle, -MaxThetaAngle, -MinThetaAngle);

			// new basis
			xAxis = new quat(vec3.UP, -PhiAngle) * binormal;
			yAxis = MathLib.Normalize(MathLib.Cross(vec3.UP, xAxis));
			zAxis = MathLib.Normalize(MathLib.Cross(xAxis, yAxis));

			// direction
			direction = (new quat(vec3.UP, -PhiAngle) * new quat(tangent, -ThetaAngle)) * binormal;

			// movement
			if (Input.IsKeyPressed(forwardKey))
				moveImpulse += xAxis;
			if (Input.IsKeyPressed(backwardKey))
				moveImpulse -= xAxis;
			if (Input.IsKeyPressed(leftKey))
				moveImpulse += yAxis;
			if (Input.IsKeyPressed(rightKey))
				moveImpulse -= yAxis;
			moveImpulse.Normalize();

			// velocity
			if (Input.IsKeyPressed(runKey))
				moveImpulse *= RunSpeed;
			else
				moveImpulse *= WalkSpeed;

			// jump
			jumpImpulse = vec3.ZERO;
			if (IsGround && Input.IsKeyDown(jumpKey))
				jumpImpulse += zAxis * JumpPower / ifps;

			// rotate velocity
			if (IsGround)
				velocity = xAxis * xSpeed + yAxis * ySpeed + zAxis * zSpeed;

			if (enableSlopeSliding)
			{
				if (IsGround && !usedAutoStepping && slopeAngle > maxStaticSlopeAngle)
				{
					vec3 z = slopeNormal;
					vec3 x = MathLib.Cross(slopeNormal, vec3.UP).Normalized;
					vec3 y = MathLib.Cross(z, x).Normalized;

					float delta = maxSlopeAngle - maxStaticSlopeAngle;
					float k = 1.0f;
					if (MathLib.Compare(delta, 0.0f) == 0)
						k = (slopeAngle - maxStaticSlopeAngle) / delta;

					float slopeSpeed = minSlidingSpeed + (maxSlidingSpeed - minSlidingSpeed) * k;

					moveImpulse += y * slopeSpeed;

					if (renderSlopeBasis)
					{
						Visualizer.RenderVector(slopePoint, slopePoint + x, new vec4(1, 0, 0, 1));
						Visualizer.RenderVector(slopePoint, slopePoint + y, new vec4(0, 1, 0, 1));
						Visualizer.RenderVector(slopePoint, slopePoint + z, new vec4(0, 0, 1, 1));
					}
				}
			}
		}
	}

	private void UpdateVelocity(float ifps)
	{
		targetSpeed = MathLib.Length(new vec2(MathLib.Dot(xAxis, moveImpulse), MathLib.Dot(yAxis, moveImpulse)));

		// save old velocity
		float oldSpeed = MathLib.Length(new vec2(MathLib.Dot(xAxis, velocity), MathLib.Dot(yAxis, velocity)));

		// integrate velocity
		velocity += moveImpulse * (Acceleration * ifps);
		velocity += Physics.Gravity * GravityMultiplier * ifps;
		velocity += jumpImpulse * ifps;

		// damping
		float currentSpeed = MathLib.Length(new vec2(MathLib.Dot(xAxis, velocity), MathLib.Dot(yAxis, velocity)));
		if (targetSpeed < MathLib.EPSILON || currentSpeed > targetSpeed)
			velocity = (xAxis * MathLib.Dot(xAxis, velocity) + yAxis * MathLib.Dot(yAxis, velocity)) * MathLib.Exp(-Damping * ifps) + zAxis * MathLib.Dot(zAxis, velocity);

		// clamp maximum velocity
		currentSpeed = MathLib.Length(new vec2(MathLib.Dot(xAxis, velocity), MathLib.Dot(yAxis, velocity)));
		if (currentSpeed > oldSpeed && currentSpeed > targetSpeed)
			velocity = (xAxis * MathLib.Dot(xAxis, velocity) + yAxis * MathLib.Dot(yAxis, velocity)) * targetSpeed / currentSpeed + zAxis * MathLib.Dot(zAxis, velocity);

		// frozen velocity
		IsFrozen = false;
		if (currentSpeed < Physics.FrozenLinearVelocity)
		{
			velocity = zAxis * MathLib.Dot(zAxis, velocity);
			IsFrozen = true;
		}
	}

	private void UpdatePosition(float ifps)
	{
		position += new vec3(velocity * ifps);
	}

	private void UpdateCollisions(float ifps)
	{
		if (!IsCollision)
			return;

		IsGround = false;
		IsCeiling = false;

		if (enableAutoStepping)
		{
			autoSteppingOffset = vec3.ZERO;

			TryMoveUp(ifps);
		}

		MoveSide(ifps);

		if (enableAutoStepping && usedAutoStepping)
		{
			TryMoveDown(ifps);

			// if autostepping was canceled use only the move side
			if (!usedAutoStepping)
				MoveSide(ifps);
		}
	}

	private void TryMoveUp(float ifps)
	{
		usedAutoStepping = false;
		lastStepHeight = 0;
		if (!IsFrozen && MaxStepHeight > 0) // when player is moving
		{
			// move (apply "position")
			dummy.Transform = GetBodyTransform();

			// find collisions with the capsule
			shape.GetCollision(contacts, 0.0f);
			if (contacts.Count == 0)
				return;

			int contactsCount = MathLib.Min(contacts.Count, contactsBufferSize);

			// find max step height
			for (int i = 0; i < contactsCount; i++)
			{
				ShapeContact c = contacts[i];

				vec3 normalXY = c.Normal - vec3.UP * MathLib.Dot(vec3.UP, c.Normal);
				vec3 velocityXY = velocity - vec3.UP * MathLib.Dot(vec3.UP, velocity);

				// skip contacts opposite to movement
				if (normalXY.Length2 > MathLib.EPSILON && velocityXY.Length2 > MathLib.EPSILON &&
					MathLib.Dot(normalXY.Normalized, velocityXY.Normalized) > 0.5f)
					continue;

				float step = MathLib.Dot(new vec3(c.Point - position), vec3.UP);
				if (lastStepHeight < step)
					lastStepHeight = step;
			}

			// apply auto stepping
			if (MinStepHeight < lastStepHeight && lastStepHeight < MaxStepHeight)
			{
				position += new vec3(zAxis * lastStepHeight);
				autoSteppingOffset += new vec3(zAxis * lastStepHeight);

				usedAutoStepping = true;
			}

			if (renderUpPassContacts)
				foreach (var c in contacts)
					Visualizer.RenderVector(c.Point, c.Point + c.Normal, new vec4(1, 0, 0, 1));
		}
	}

	private void MoveSide(float ifps)
	{
		vec3 tangent = vec3.ZERO;
		vec3 binormal = vec3.ZERO;

		for (int i = 0; i < collisionIterations; i++)
		{
			// move (apply "position")
			dummy.Transform = GetBodyTransform();

			// find collisions with the capsule
			shape.GetCollision(contacts, 0.0f);

			// in case of a large number of contacts, we use one iteration to avoid performance degradation
			if (contacts.Count > heavyContactsCount)
				i = collisionIterations - 1;

			if (contacts.Count == 0)
				break;

			int contactsCount = MathLib.Min(contacts.Count, contactsBufferSize);

			float inumContacts = 1.0f / MathLib.Itof(contactsCount);

			float maxSlopeAngle = -MathLib.INFINITY;

			// solving collisions
			for (int j = 0; j < contactsCount; j++)
			{
				ShapeContact c = contacts[j];

				FixContact(ref c);

				float normalSpeed = 0.0f;

				if (IsFrozen)
				{
					position += zAxis * c.Depth * inumContacts * MathLib.Dot(zAxis, c.Normal);
					autoSteppingOffset += zAxis * c.Depth * inumContacts * MathLib.Dot(zAxis, c.Normal);

					normalSpeed = MathLib.Dot(zAxis, velocity);
					velocity -= zAxis * normalSpeed;
				}
				else
				{
					position += c.Normal * c.Depth * inumContacts;
					autoSteppingOffset += c.Normal * c.Depth * inumContacts;
					IsFrozen = false;

					normalSpeed = MathLib.Dot(c.Normal, velocity);
					velocity -= c.Normal * normalSpeed;
				}

				// friction
				if (targetSpeed < MathLib.EPSILON)
				{
					Geometry.OrthoBasis(c.Normal, out tangent, out binormal);

					float tangentSpeed = MathLib.Dot(tangent, velocity);
					float binormalSpeed = MathLib.Dot(binormal, velocity);

					if (MathLib.Abs(tangentSpeed) > MathLib.EPSILON || MathLib.Abs(binormalSpeed) > MathLib.EPSILON)
					{
						float frictionSpeed = MathLib.Max(-normalSpeed, 0.0f) * friction * MathLib.Rsqrt(tangentSpeed * tangentSpeed + binormalSpeed * binormalSpeed);
						frictionSpeed = MathLib.Clamp(frictionSpeed, -1.0f, 1.0f);

						velocity -= tangent * tangentSpeed * frictionSpeed;
						velocity -= binormal * binormalSpeed * frictionSpeed;
					}
				}

				if (MathLib.Dot(c.Normal, vec3.UP) > 0.5f && MathLib.Dot(new vec3(c.Point - shape.BottomCap), vec3.UP) < 0.0f)
				{
					IsGround = true;

					float angle = MathLib.GetAngle(vec3.UP, c.Normal);
					if (angle > maxSlopeAngle && 0 < angle && angle < 90.0f)
					{
						slopeNormal = c.Normal;
						slopePoint = c.Point;
						slopeAngle = angle;
						maxSlopeAngle = angle;
					}
				}

				if (MathLib.Dot(c.Normal, vec3.UP) < -0.5f && MathLib.Dot(new vec3(c.Point - shape.TopCap), vec3.UP) > 0.0f)
					IsCeiling = true;
			}

			if (renderSidePassContacts)
				foreach (var c in contacts)
					Visualizer.RenderVector(c.Point, c.Point + c.Normal, new vec4(0, 1, 1, 1));
		}
	}

	private void TryMoveDown(float ifps)
	{
		vec3 moveImpulseXY = moveImpulse - vec3.UP * MathLib.Dot(vec3.UP, moveImpulse);
		Vec3 pos1 = position + (moveImpulseXY.Normalized + vec3.UP) * shape.Radius;
		Vec3 pos2 = pos1 - vec3.UP * shape.Radius;

		WorldIntersectionNormal intersection = new WorldIntersectionNormal();
		Unigine.Object hitObj = World.GetIntersection(pos1, pos2, contactsIntersectionMask, intersection);
		if (hitObj)
		{
			float angle = MathLib.GetAngle(vec3.UP, intersection.Normal);
			if (autoSteppingCancelAngle < angle && angle < 90.0f)
			{
				position -= autoSteppingOffset;
				usedAutoStepping = false;
				return;
			}
		}

		vec3 velocityXY = velocity - vec3.UP * MathLib.Dot(vec3.UP, velocity);
		pos1 = position + (velocityXY.Normalized + vec3.UP) * shape.Radius;
		pos2 = pos1 - vec3.UP * shape.Radius;
		hitObj = World.GetIntersection(pos1, pos2, contactsIntersectionMask, intersection);
		if (hitObj)
		{
			float angle = MathLib.GetAngle(vec3.UP, intersection.Normal);
			if (autoSteppingCancelAngle < angle && angle < 90.0f)
			{
				position -= autoSteppingOffset;
				usedAutoStepping = false;
				return;
			}
		}

		// this correction allows to avoid jittering on large stairs
		if (lastStepHeight > shape.Radius)
		{
			if (GravityMultiplier > 1.0f)
				lastStepHeight = shape.Radius / GravityMultiplier - Physics.PenetrationTolerance;
			else
				lastStepHeight = shape.Radius - Physics.PenetrationTolerance;
		}

		// try to drop down the player
		position -= new vec3(zAxis * lastStepHeight);
		autoSteppingOffset -= new vec3(zAxis * lastStepHeight);

		// move (apply "position")
		dummy.Transform = GetBodyTransform();

		// find collisions with the capsule
		shape.GetCollision(contacts, 0.0f);
		if (contacts.Count == 0)
			return;

		int contactsCount = MathLib.Min(contacts.Count, contactsBufferSize);

		float inumContacts = 1.0f / MathLib.Itof(contactsCount);

		// push up (if collisions exists)
		for (int i = 0; i < contactsCount; i++)
		{
			ShapeContact c = contacts[i];

			position += new vec3(zAxis * (MathLib.Max(c.Depth, 0.0f) * inumContacts * MathLib.Dot(zAxis, c.Normal)));

			if (MathLib.Dot(c.Normal, vec3.UP) > 0.5f && MathLib.Dot(new vec3(c.Point - shape.BottomCap), vec3.UP) < 0.0f)
				IsGround = true;
		}

		if (renderDownPassContacts)
			foreach (var c in contacts)
				Visualizer.RenderVector(c.Point, c.Point + c.Normal, new vec4(0, 1, 0, 1));
	}

	private void UpdateTransform()
	{
		// ortho basis
		vec3 tangent, binormal;
		Geometry.OrthoBasis(vec3.UP, out tangent, out binormal);

		// decompose transformation
		position = node.WorldPosition - zAxis * (collisionHeight + 2.0f * CollisionRadius);
		direction = MathLib.Normalize(new vec3(-node.WorldTransform.AxisZ));

		// decompose direction
		// in this case don't use properties
		phiAngle = MathLib.Atan2(MathLib.Dot(direction, tangent), MathLib.Dot(direction, binormal)) * MathLib.RAD2DEG;
		thetaAngle = MathLib.Acos(MathLib.Clamp(MathLib.Dot(direction, vec3.UP), -1.0f, 1.0f)) * MathLib.RAD2DEG - 90.0f;

		// object transformation
		playerBody.WorldTransform = GetBodyTransform();

		// set camera
		camera.Modelview = GetModelview();
	}

	private void FlushTransform()
	{
		node.WorldTransform = new Mat4(MathLib.SetTo(position, position + new vec3(direction - vec3.UP * MathLib.Dot(direction, vec3.UP)), vec3.UP) * new Mat4(MathLib.RotateX(-90.0f) * MathLib.RotateZ(90.0f)));
		camera.Modelview = GetModelview();
	}

	private Mat4 GetBodyTransform()
	{
		vec3 center = new vec3(position) + vec3.UP * (shape.Height * 0.5f + shape.Radius);
		return new Mat4(MathLib.SetTo(center, center + new vec3(direction - vec3.UP * MathLib.Dot(direction, vec3.UP)), vec3.UP) * new mat4(MathLib.RotateX(-90.0f) * MathLib.RotateZ(90.0f)));
	}

	private Mat4 GetModelview()
	{
		vec3 eye = new vec3(position) + vec3.UP * CameraHeight;
		return new Mat4(MathLib.LookAt(eye, eye + new vec3(direction), vec3.UP));
	}

	/// <summary>
	/// This function fixed contacts only with external objects for capsule
	/// </summary>
	private void FixContact(ref ShapeContact contact)
	{
		Vec3 firstPoint = Vec3.ZERO;

		// set first point for intersection
		// it's bottom or top sphere of capsule
		// or cylinder part of capsule
		if (contact.Point.z < shape.BottomCap.z)
			firstPoint = shape.BottomCap;
		else if (shape.BottomCap.z <= contact.Point.z && contact.Point.z <= shape.TopCap.z)
			firstPoint = new Vec3(shape.TopCap.x, shape.TopCap.y, contact.Point.z);
		else
			firstPoint = shape.TopCap;

		// try get contact object
		WorldIntersectionNormal normalIntersection = new WorldIntersectionNormal();
		var hitObj = World.GetIntersection(firstPoint, contact.Point, contactsIntersectionMask, normalIntersection);
		if (hitObj)
		{
			// get real distance to contact
			float distance = (float)(normalIntersection.Point - firstPoint).Length;
			if (distance < shape.Radius)
			{
				// set correct parameters for contact
				contact.Point = normalIntersection.Point;
				contact.Depth = shape.Radius - distance;
				contact.Normal = new vec3((firstPoint - contact.Point).Normalize());
			}
			else
			{
				// contact outside capsule
				contact.Point = Vec3.ZERO;
				contact.Depth = 0.0f;
				contact.Normal = vec3.ZERO;
			}
		}
		// check contact with horizontal plane, because intersection can't detect it
		else if (MathLib.Dot(vec3.UP, contact.Normal) < MathLib.EPSILON)
		{
			if (contact.Object)
			{
				vec3 scale = contact.Object.WorldScale;

				// check object scale
				if (MathLib.Compare(scale.x, 1.0f) != 1 ||
					MathLib.Compare(scale.y, 1.0f) != 1 ||
					MathLib.Compare(scale.z, 1.0f) != 1)
				{
					// this value avoids jittering
					contact.Depth = MathLib.Max(Physics.PenetrationTolerance, 0.01f);
				}
			}
		}
		else
		{
			// capsule does not contact with this object
			contact.Point = Vec3.ZERO;
			contact.Depth = 0.0f;
			contact.Normal = vec3.ZERO;
		}
	}
}
