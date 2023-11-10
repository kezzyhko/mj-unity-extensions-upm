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

        [SerializeField]
        MjBody pupeteeredJoint4Debug;

        public MjBody PupeteeredBody {  set => pupeteeredJoint4Debug = value; }

        public MjBody PairedBody { get => pairedBody; set => pairedBody = value; }
   
        Vector3 prevPosition;
        Quaternion prevRotation;


        Quaternion prevLocalRotation  = Quaternion.identity;
        Quaternion currentLocalRotation = Quaternion.identity;

        private Vector3 Position => transform.position;
        private Quaternion Rotation => transform.rotation;
        private Quaternion LocalRotation => transform.localRotation;




        private float fs;

        private Vector3 Velocity => (Position - prevPosition)*fs;
        
        private Vector3 AngularVelocity => Utils.RotationVel(Rotation, prevRotation, fs);
        private Vector3 LocalAngularVelocity => Utils.RotationVel(currentLocalRotation, prevLocalRotation, fs); //QuaternionLocalVel();

     


        [SerializeField]
        private Vector3 localForward = Vector3.forward;

        FiniteDifferenceBodyKinematics kinematics;

        public static
        Vector3 offset4debug = new Vector3(0.05f, 0.0f, 0.0f);




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
        
        }


        public void FixedUpdate()
        {
            Step();
        }


        public void Step()
        {
            prevPosition = transform.position;
            prevRotation = transform.rotation;

            prevLocalRotation = currentLocalRotation;
            currentLocalRotation = LocalRotation;
    

    }

    public IKinematic GetIKinematic()
        {
            if(kinematics == null) kinematics = new FiniteDifferenceBodyKinematics(this);
            return kinematics;
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying) return;
        
           // Draw2();

            Draw();
        }

        public void Draw()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, Rotation * Vector3.forward * 0.015f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, Rotation * Vector3.up * 0.015f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, Rotation * Vector3.right * 0.015f);
  

            Gizmos.color = Color.grey;

            var parent = pupeteeredJoint4Debug.GetComponentInParent<MjBody>();

            Gizmos.DrawRay(Position+0.005f*Vector3.up , (!parent ? LocalAngularVelocity : parent.GetTransformMatrix().MultiplyVector(LocalAngularVelocity)) * 0.2f);


            Gizmos.color = Color.black;
            Gizmos.DrawRay(Position + 0.005f * Vector3.up,LocalAngularVelocity * 0.2f);


            if (pupeteeredJoint4Debug != null)
            {
                Gizmos.color = Color.blue;                    
                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();
                Gizmos.DrawRay(Position, (!parent ? pupetKin.LocalAngularVelocity : parent.GetTransformMatrix().MultiplyVector(pupetKin.LocalAngularVelocity)) * 0.2f);

                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(Position, pupetKin.LocalAngularVelocity * 0.2f);

            }


          

        }


        public void Draw2()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.forward * 0.15f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.up * 0.15f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.right * 0.15f);
     

            if (pupeteeredJoint4Debug != null)
            {
                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.forward * 0.15f);
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.up * 0.15f);
                Gizmos.color = Color.grey;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.right * 0.15f);



            }

        }




        private class FiniteDifferenceBodyKinematics : IKinematic
        {
            MjFiniteDifferenceBody component;
            float mass;
            Vector3 inertiaLocalPos;
            Matrix4x4 inertiaRelMatrix;
            IKinematic parentKinematics;
            bool isRoot;

            public FiniteDifferenceBodyKinematics(MjFiniteDifferenceBody component) 
            { 
                this.component = component;


                var parent = this.component.transform.parent.GetComponentInParent<MjFiniteDifferenceBody>();  //Need to call at parent as GetComponentInParent is inclusive of the start transform.

                isRoot = !parent;
                if (parent) parentKinematics = parent.GetIKinematic();  //This doesn't create an endless loop as there's an exit condition.

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

            public Vector3 LocalAngularVelocity => component.LocalAngularVelocity;

            public float Mass => mass;

            public Vector3 CenterOfMass => Matrix4x4.TRS(Position,
                                                 Rotation,
                                                 Vector3.one).MultiplyPoint3x4(inertiaLocalPos);

            public Matrix4x4 TransformMatrix => Matrix4x4.TRS(Position,
                                                              Rotation,
                                                              Vector3.one) * inertiaRelMatrix;

            public int index => -component.pairedBody.MujocoId;

            public Quaternion Rotation => component.Rotation;

            public Quaternion LocalRotation => component.LocalRotation;

            public Vector3 Position => component.Position;

            public Vector3 LocalVelocity => isRoot? Velocity : Velocity - parentKinematics.Velocity;

        

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