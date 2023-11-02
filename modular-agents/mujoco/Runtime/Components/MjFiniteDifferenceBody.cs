using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using ModularAgents.MotorControl.CircularBuffer;
using System.Linq;
using Mujoco.Extensions;

namespace ModularAgents.Kinematic.Mujoco
{
    /// <summary>
    /// A MocapBody that can provide 6 DOF velocity estimates based on finite differences.
    /// </summary>

    public class MjFiniteDifferenceBody : MonoBehaviour, IKinematicProvider, IFiniteDifferenceComponent
    {
        [SerializeField]
        MjBody pairedBody;
        public MjBody PairedBody { get => pairedBody; set => pairedBody = value; }
        IKinematic pairedKinematics;


        Vector3 prevPosition;
        Quaternion prevRotation;

        private Vector3 Position => transform.position;
        private Quaternion Rotation => transform.rotation;

        private float fs;

        private Vector3 Velocity => (Position - prevPosition)*fs;
        private Vector3 AngularVelocity => Utils.QuaternionError(Rotation, prevRotation)*fs;

        [SerializeField]
        private Vector3 localForward = Vector3.forward;

        FiniteDifferenceBodyKinematics kinematics;

        private void Awake()
        {
            Step();
        }

        private void Start()
        {
            MjState.ExecuteAfterMjStart(MjInitialize);
        }

        private void MjInitialize()
        {
            fs = 1 / Time.fixedDeltaTime;
            pairedKinematics = pairedBody.transform.GetIKinematic();
        }

        public void Step()
        {
            prevPosition = transform.position;
            prevRotation = transform.rotation;
        }

        public IKinematic GetIKinematic()
        {
            if(kinematics == null) kinematics = new FiniteDifferenceBodyKinematics(this);
            return kinematics;
        }

        private class FiniteDifferenceBodyKinematics : IKinematic
        {
            MjFiniteDifferenceBody component;
            float mass;
            Vector3 inertiaLocalPos;
            Matrix4x4 inertiaRelMatrix;
            IKinematic parentKinematics;
            bool isRoot;

            public FiniteDifferenceBodyKinematics(MjFiniteDifferenceBody comoponent) 
            { 
                this.component = comoponent;
                var parent = component.GetComponentInParent<MjFiniteDifferenceBody>();
                if (parent) parentKinematics = parent.GetIKinematic();
                isRoot = !parent;
                MjState.ExecuteAfterMjStart(MjInitialize);
            }

            private void MjInitialize()
            {
                mass = component.pairedBody.GetMass();
                inertiaLocalPos = component.pairedBody.GetLocalCenterOfMass();
                inertiaRelMatrix = component.pairedBody.GetInertiaToBodyMatrix();
            }
            public Vector3 Velocity => component.Velocity;

            public Vector3 AngularVelocity => component.AngularVelocity;

            public float Mass => mass;

            public Vector3 CenterOfMass => Matrix4x4.TRS(Position,
                                                 Rotation,
                                                 Vector3.one).MultiplyPoint3x4(inertiaLocalPos);

            public Matrix4x4 TransformMatrix => Matrix4x4.TRS(Position,
                                                              Rotation,
                                                              Vector3.one) * inertiaRelMatrix;

            public int index => -component.pairedBody.MujocoId;

            public Quaternion Rotation => component.Rotation;

            public Quaternion LocalRotation => isRoot? Rotation : Quaternion.Inverse(parentKinematics.Rotation) * Rotation;

            public Vector3 Position => component.Position;

            public Vector3 LocalVelocity => isRoot? Velocity : Velocity - parentKinematics.Velocity;

            public Vector3 LocalAngularVelocity => isRoot ? AngularVelocity : AngularVelocity - parentKinematics.AngularVelocity;

            public string Name => component.name;

            public GameObject gameObject => component.gameObject;

            public Vector3 Forward => Rotation * component.localForward;

            public Vector3 GetPointVelocity(Vector3 worldPoint)
            {
                return Vector3.Cross((worldPoint - CenterOfMass), AngularVelocity) + Velocity;
            }

            /// <summary>
            /// Follows Unity convention of methods with the same name
            /// </summary>
            /// <param name="localPoint">Point local to the inertial frame of the body (i.e. with the CoM in the centre)</param>
            /// <returns>Global velocity of the point</returns>
            public Vector3 GetRelativePointVelocity(Vector3 localPoint)
            {
                return Vector3.Cross(TransformMatrix.MultiplyVector(localPoint), AngularVelocity) + Velocity;
            }
        }
    }

    public interface IKinematicProvider
    {
        public IKinematic GetIKinematic();
    }

    public interface IFiniteDifferenceComponent
    {
        public void Step();
    }
}