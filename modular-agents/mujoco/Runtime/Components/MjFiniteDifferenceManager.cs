using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mujoco.Extensions;
using UnityEngine;

public class MjFiniteDifferenceManager :MonoBehaviour// : TrainingEventHandler
{
    [SerializeField]
    MjFreeJoint pairedRootJoint;

  //  [SerializeField]
  //  public bool useInPupeteering;

    List<MjFiniteDifferenceJoint> orderedJoints;


    public MjFreeJoint Root => pairedRootJoint;

    public MjFiniteDifferenceBody animationRoot;


    [SerializeField]
    Animator animator;
    public Animator Animator => animator;

    MjFiniteDifferenceBody[] managedComponents;

  

    private void Start()
    {
      managedComponents =  gameObject.transform.GetComponentsInChildren<MjFiniteDifferenceBody>();
        
        //to make sure they all have the right tracking of their parents we do:
       foreach (MjFiniteDifferenceBody comp in managedComponents)
            comp.GetIKinematic();
       var fdJoints = GetComponentsInChildren<MjFiniteDifferenceJoint>();
       orderedJoints = pairedRootJoint.GetComponentInParent<MjBody>().GetTopDownOrderedComponents<MjBaseJoint>().Select(j => fdJoints.First(fdj => fdj.PairedJoint == j)).ToList();
      
        
        
      
    }

    public unsafe void CopyStateToPairedTree()
    {

        MjState.TeleportMjRoot(pairedRootJoint, animationRoot.transform.position, animationRoot.transform.rotation);

      
        foreach (MjFiniteDifferenceJoint mfdj in orderedJoints)
        { 
            mfdj.Reset();
        
        }
      

    }

    
    public void Step()
    {
     
       foreach (var component in managedComponents)
        {
            component.Step();
        }

    }

    public unsafe void ForwardKinematics() 
    {
        MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);

    }

    
    public unsafe void FixedUpdate()
    {
        Step();
    }
    



    public void JumpToNormalizedTime(float normalizedTime)
    {
        
        AnimatorStateInfo stateInfo = animator.GetCurrentAnimatorStateInfo(0);
        var startTime = Mathf.Clamp01(normalizedTime - Time.fixedDeltaTime/stateInfo.length);
        animator.Play(stateInfo.fullPathHash, -1, startTime);
        animator.Update(0);
     //   Step();
    }


}
