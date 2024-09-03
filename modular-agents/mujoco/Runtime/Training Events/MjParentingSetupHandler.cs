using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Mujoco;
using Mujoco.Extensions;
using System.Data;
using ModularAgents.Kinematic;


namespace ModularAgents.TrainingEvents 
{ 
    public class MjParentingSetupHandler : MjBasicSetupHandler
    {
        [SerializeField]
        Transform overrideAnimationRootParentTransform;
        protected override void SetupKineticChain()
        {
            referenceAnimationParent = overrideAnimationRootParentTransform;
            base.SetupKineticChain();
        }

    }


}