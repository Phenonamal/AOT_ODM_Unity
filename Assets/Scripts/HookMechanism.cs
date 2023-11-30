using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

public class HookMechanism : MonoBehaviour
{
    [Header("Grapple Settings")]
    [SerializeField] private LayerMask grappleMask;
    [SerializeField] private float maxDistance = 100f;
    [SerializeField] private float springForce = 100f;
    [SerializeField] private float upwardForce = 30f;
    [SerializeField] private float rotationSpeed = 60f;
    [SerializeField] private float maxSwingAngle = 120f;
    [SerializeField] private float reelInSpeed = 120f;
    [SerializeField] private float boostCooldown = 0.2f;
    [SerializeField] private float boostDuration = 1f;
    [SerializeField] private float boostMultiplier = 2f;
    [SerializeField] private float dampingRotation = 20f;
    [SerializeField] private float smoothVelocityFactor = 0.1f;
    [SerializeField] private float momentumRetentionTime = 0.5f;
    [SerializeField] private float driftSpeed = 5f;
    [SerializeField] private float groundedRaycastDistance = 0.1f;

    [Header("Visual Effects")]
    [SerializeField] private LineRenderer[] grappleLines;
    [SerializeField] private ParticleSystem boostParticles;
    [SerializeField] private AudioSource boostSound;

    [Header("Components")]
    [SerializeField] private Transform[] gunTips;
    [SerializeField] private Transform player;
    [SerializeField] private Camera camera;

    private List<SpringJoint> joints = new List<SpringJoint>();
    private bool[] isGrappling;
    private bool[] isReleasingGrapple;
    private Vector3[] grapplePoints;
    private float[] nextBoostTimes;
    private bool originalGravityState;
    private Vector3[] lastReelInDirections;
    private float[] releaseTimes;
    private Vector3[] smoothVelocities;
    private bool isBoosting;
    private float boostStartTime;
    private bool isDrifting;
    private float driftDirection;

    private Rigidbody playerRigidbody;

    [SerializeField] private InputAction[] grappleActions;
    [SerializeField] private InputAction boostAction;
    [SerializeField] private InputAction driftAction;

    private void Awake()
    {
        int grappleCount = 2; // Set the number of grappling hooks
        isGrappling = new bool[grappleCount];
        isReleasingGrapple = new bool[grappleCount];
        grapplePoints = new Vector3[grappleCount];
        nextBoostTimes = new float[grappleCount];
        lastReelInDirections = new Vector3[grappleCount];
        releaseTimes = new float[grappleCount];
        smoothVelocities = new Vector3[grappleCount];

        for (int i = 0; i < grappleCount; i++)
        {
            grappleActions[i].Enable();
            int index = i; // Capture the index variable
            grappleActions[i].started += _ => StartGrapple(index);
            grappleActions[i].canceled += _ => ReleaseGrapple(index);
        }

        boostAction.Enable();
        boostAction.performed += _ => StartBoost();

        driftAction.Enable();
        driftAction.performed += ctx => isDrifting = ctx.ReadValue<float>() != 0f;
        driftAction.canceled += _ => isDrifting = false;

        playerRigidbody = player.GetComponent<Rigidbody>();
    }

    private void OnDestroy()
    {
        for (int i = 0; i < grappleActions.Length; i++)
        {
            grappleActions[i].Disable();
        }

        boostAction.Disable();
        driftAction.Disable();
    }

    private void Update()
    {
        for (int i = 0; i < isGrappling.Length; i++)
        {
            if (isGrappling[i])
            {
                HandleGrappleBehavior(i);
            }
            else if (isReleasingGrapple[i])
            {
                HandleReleaseGrappleBehavior(i);
            }
        }

        if (isBoosting && isGrappling[0]) // Only boost while grappling
        {
            HandleBoost();
        }

        if (isDrifting)
        {
            HandleDrift();
        }
    }

    private void HandleGrappleBehavior(int index)
    {
        ApplySpringForce(index);
        RotatePlayerTowardsGrapplePoint(index);
        LimitSwingAngle(index);
        ApplyUpwardForce(index);
        HandleBoostEffect(index);
        ReelInTowardsGrapplePoint(index);
    }

    private void HandleReleaseGrappleBehavior(int index)
    {
        RetainMomentum(index);
    }

    private void HandleBoost()
    {
        if (Time.time - boostStartTime < boostDuration)
        {
            BoostPlayer();
        }
        else
        {
            StopBoost();
        }
    }

    private void BoostPlayer()
    {
        Vector3 boostDirection = player.forward;
        float boostSpeed = reelInSpeed * boostMultiplier;
        playerRigidbody.AddForce(boostDirection * boostSpeed, ForceMode.Acceleration);
    }

    private void StopBoost()
    {
        isBoosting = false;
        boostParticles.Stop();
        boostSound.Stop();
    }

    private void HandleDrift()
    {
        Vector3 driftDir = new Vector3(driftDirection, 0f, 0f);
        driftDir.Normalize();
        driftDir = player.transform.TransformDirection(driftDir);
        playerRigidbody.AddForce(driftDir * driftSpeed, ForceMode.Acceleration);
    }

    private void ApplySpringForce(int index)
    {
        Vector3 grappleDirection = (grapplePoints[index] - player.position).normalized;
        playerRigidbody.AddForce(grappleDirection * springForce, ForceMode.Acceleration);
    }

    private void RotatePlayerTowardsGrapplePoint(int index)
    {
        Quaternion targetRotation = Quaternion.LookRotation(grapplePoints[index] - player.position, Vector3.up);
        player.rotation = Quaternion.Slerp(player.rotation, targetRotation, rotationSpeed * Time.deltaTime);
    }

    private void LimitSwingAngle(int index)
    {
        Vector3 playerToGrapple = grapplePoints[index] - player.position;
        float angle = Vector3.Angle(player.forward, playerToGrapple);

        if (angle > maxSwingAngle)
        {
            Vector3 newForward = Vector3.RotateTowards(player.forward, playerToGrapple, maxSwingAngle * Mathf.Deg2Rad, 0f);
            player.rotation = Quaternion.LookRotation(newForward, Vector3.up);
        }
    }

    private void ApplyUpwardForce(int index)
    {
        playerRigidbody.AddForce(Vector3.up * upwardForce, ForceMode.Acceleration);
    }

    private void HandleBoostEffect(int index)
    {
        if (Time.time > nextBoostTimes[index])
        {
            TriggerBoostEffect();
        }
        else
        {
            StopBoostEffect();
        }
    }

    private void TriggerBoostEffect()
    {
        if (!boostParticles.isPlaying)
        {
            boostParticles.Play();
        }

        if (!boostSound.isPlaying)
        {
            boostSound.Play();
        }
    }

    private void StopBoostEffect()
    {
        if (boostParticles.isPlaying)
        {
            boostParticles.Stop();
        }

        if (boostSound.isPlaying)
        {
            boostSound.Stop();
        }
    }

    private void ReelInTowardsGrapplePoint(int index)
    {
        Vector3 reelInDirection = (grapplePoints[index] - player.position).normalized;
        playerRigidbody.AddForce(reelInDirection * reelInSpeed, ForceMode.Acceleration);
        lastReelInDirections[index] = reelInDirection;

        // Smooth out the velocity to avoid sudden changes
        playerRigidbody.velocity = Vector3.SmoothDamp(playerRigidbody.velocity, Vector3.zero, ref smoothVelocities[index], smoothVelocityFactor);
    }

    private void RetainMomentum(int index)
    {
        if (Time.time - releaseTimes[index] < momentumRetentionTime)
        {
            playerRigidbody.AddForce(lastReelInDirections[index] * springForce * 0.5f, ForceMode.Acceleration);
        }
        else
        {
            isReleasingGrapple[index] = false;
        }
    }

    private void StartGrapple(int index)
    {
        if (!isGrappling[index] && Time.time > nextBoostTimes[index])
        {
            RaycastHit hit;
            Ray ray = camera.ScreenPointToRay(new Vector3(Screen.width / 2f, Screen.height / 2f, 0f));

            if (Physics.Raycast(ray, out hit, maxDistance, grappleMask))
            {
                InitializeGrapple(index, hit.point);
            }
        }
    }

    private void InitializeGrapple(int index, Vector3 hitPoint)
    {
        grapplePoints[index] = hitPoint;
        joints.Add(player.gameObject.AddComponent<SpringJoint>());
        originalGravityState = playerRigidbody.useGravity;
        playerRigidbody.useGravity = false; // Disable gravity
        ConfigureSpringJoint(index);
        grappleLines[index].positionCount = 2;
        isGrappling[index] = true;
        nextBoostTimes[index] = Time.time + boostCooldown;
    }

    private void ConfigureSpringJoint(int index)
    {
        SpringJoint joint = joints[joints.Count - 1];
        float distanceFromPoint = Vector3.Distance(player.position, grapplePoints[index]);

        joint.autoConfigureConnectedAnchor = false;
        joint.connectedAnchor = grapplePoints[index];
        joint.maxDistance = distanceFromPoint * 0.8f;
        joint.minDistance = distanceFromPoint * 0.1f;
        joint.spring = springForce;
        joint.damper = 7f;
        joint.massScale = 4.5f;
    }

    private void ReleaseGrapple(int index)
    {
        if (isGrappling[index])
        {
            isGrappling[index] = false;
            isReleasingGrapple[index] = true;
            releaseTimes[index] = Time.time;
            Destroy(joints[joints.Count - 1]); // Remove the joint
            joints.RemoveAt(joints.Count - 1); // Remove the joint from the list
            grappleLines[index].positionCount = 0;
            playerRigidbody.useGravity = originalGravityState; // Re-enable gravity
            StopBoostEffect();
        }
    }

    private void LateUpdate()
    {
        DrawRopes();
    }

    private void DrawRopes()
    {
        for (int i = 0; i < grappleLines.Length; i++)
        {
            if (isGrappling[i])
            {
                grappleLines[i].SetPosition(0, gunTips[i].position);
                grappleLines[i].SetPosition(1, grapplePoints[i]);
            }
            else
            {
                grappleLines[i].positionCount = 0;
            }
        }
    }

    private void StartBoost()
    {
        if (!isBoosting && Time.time > nextBoostTimes[0])
        {
            isBoosting = true;
            boostStartTime = Time.time;
        }
    }
}
