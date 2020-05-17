using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using System.Runtime.InteropServices;

public class FreeFlightController : MonoBehaviour {
    [Tooltip("Enable/disable rotation control. For use in Unity editor only.")]
    public bool rotationEnabled = true;

    [Tooltip("Enable/disable translation control. For use in Unity editor only.")]
    public bool translationEnabled = true;

    //private WebVRDisplayCapabilities capabilities;

    [Tooltip("Mouse sensitivity")]
    public float mouseSensitivity = 1f;

    [Tooltip("Straffe Speed")]
    public float straffeSpeed = 3f;

    [DllImport("__Internal")]
    private static extern void openWindow(string url);
    private string exitUrl = "www.oxfordigitalab.com";


    private float minimumX = -360f;
    private float maximumX = 360f;

    private float minimumY = -90f;
    private float maximumY = 90f;

    private float rotationX = 0f;
    private float rotationY = 0f;

    Quaternion originalRotation;

    private GameObject currentObject;
    private float currentDistance;

    private GameObject activeMesh;
    //private Text txtObject;

    private Vector3 screenPoint;
    private Vector3 offset;

    // labeling the playspace
    public GameObject LabelObject;
    private bool isHudBusy = false;

    // cursor objects
    public Texture2D cursorForScene;
    public Texture2D cursorForObjects;
    private Vector2 hotspot = new Vector2(10, 5);
    private readonly CursorMode cMode = CursorMode.ForceSoftware;

    private bool isDragging = false;
    private float invertMouse = 1.0f;

    public bool isGameMode = true;

    /*

    bool inDesktopLike {
        get {
            return capabilities.hasExternalDisplay;
        }
    }
    */

    void Start()
    {
        originalRotation = transform.localRotation;

        // set initial (game mode) cursor attr
        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;
    }


    private Quaternion GetCameraRotationFromMouse(float sensitivity)
    {
        rotationX += (Input.GetAxis("Mouse X") * invertMouse) * sensitivity;
        rotationY += (Input.GetAxis("Mouse Y") * invertMouse) * sensitivity;
        rotationX = ClampAngle(rotationX, minimumX, maximumX);
        rotationY = ClampAngle(rotationY, minimumY, maximumY);

        Quaternion xQuaternion = Quaternion.AngleAxis(rotationX, Vector3.up);
        Quaternion yQuaternion = Quaternion.AngleAxis(rotationY, Vector3.left);
        Quaternion newRot = originalRotation * xQuaternion * yQuaternion;
        return newRot;
    }

    void Update() {
        if (translationEnabled)
        {
            float x = Input.GetAxis("Horizontal") * Time.deltaTime * straffeSpeed;
            float z = Input.GetAxis("Vertical") * Time.deltaTime * straffeSpeed;

            float elev = Input.GetAxis("Jump") * Time.deltaTime * straffeSpeed;

            transform.Translate(x, elev, z);

            if (isDragging)
            {
                activeMesh.transform.position = GetDraggingPoint();
            }
        }

        if (rotationEnabled && isGameMode)
        {
            Quaternion camQuat = GetCameraRotationFromMouse(mouseSensitivity);
            StartCoroutine(RotateCamera(camQuat, mouseSensitivity));
        }
        else if (rotationEnabled && !isGameMode)
        {
            if (Input.GetMouseButton(0))
            {
                Quaternion camQuat = GetCameraRotationFromMouse(mouseSensitivity * 3);
                StartCoroutine(RotateCamera(camQuat, mouseSensitivity));
            }
        }

        if (!XRDevice.isPresent)
        {
            currentObject = ScreenRaycast();

            if (currentObject != null && currentObject.layer >= 9)
            {
                // set active mesh (for object and furniture) and update UI
                SetActiveMesh();


                string meshName;
                if (activeMesh != null) { meshName = activeMesh.name; }
                else { meshName = ""; }


                Cursor.SetCursor(cursorForObjects, hotspot, cMode);
            }
            else
            {
                activeMesh = null;
                Cursor.SetCursor(cursorForScene, hotspot, cMode);
            }

        }
    }


    void OnGUI()
    {
        if (!XRDevice.isPresent)
        {
            Event e = Event.current;
            if (e.isMouse && activeMesh != null)
            {
                
                
                if (e.clickCount == 2 && e.button == 0)
                {
                    DoubleLeftClick();
                }
                
                else if (e.type == EventType.MouseUp && e.button == 1)
                {
                    RightClick();
                }
                else if (e.type == EventType.MouseDown && e.button == 0 && currentObject.layer == 10)
                {
                    // only object in layer 10 are draggable
                    screenPoint = Camera.main.WorldToScreenPoint(activeMesh.transform.position);
                    if (isGameMode)
                    {
                        offset = activeMesh.transform.position - Camera.main.ViewportToWorldPoint(new Vector3(0.5f, 0.5f, screenPoint.z));
                    }
                    else
                    {
                        offset = activeMesh.transform.position - Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, screenPoint.z));
                    }

                    if (activeMesh.GetComponent<Rigidbody>())
                    {
                        activeMesh.GetComponent<Rigidbody>().isKinematic = true;
                    }

                    isDragging = true;
                }
                else if (e.type == EventType.MouseDrag && e.button == 0 && currentObject.layer == 10)
                {
                    if (isDragging)
                    {
                        activeMesh.transform.position = GetDraggingPoint();
                    }
                }
                else if (e.type == EventType.MouseUp && e.button == 0)
                {
                    if (isDragging)
                    {
                        isDragging = false;
                        
                        Rigidbody newRB = activeMesh.GetComponent<Rigidbody>(); 
                        if (newRB != null)
                        {
                            //newRB.isKinematic = false;
                            //newRB.useGravity = true;

                        }
                        
                    }
                }
            }
            
            if (e.type == EventType.KeyDown)
            {

                if (e.keyCode == KeyCode.M)
                {
                    isGameMode = !isGameMode;
                    
                    // toggle cursor visibility
                    Cursor.visible = !isGameMode;

                    // toggle crosshair visibility
                    GameObject crosshair = GameObject.FindGameObjectWithTag("crosshair");
                    int isGamey = (isGameMode ? 1 : 0) * 255;
                    crosshair.GetComponent<SimpleCrosshair>().SetColor(CrosshairColorChannel.ALPHA, isGamey, true);

                    if (isGameMode) { Cursor.lockState = CursorLockMode.Locked; }
                    else { Cursor.lockState = CursorLockMode.None; }
                }

            }
            
        }
    }


    private void SetActiveMesh()
    {
        // priority: 
        // Rigidbody of self, parent, child, 
        // then Collider of self, parent, child, 
        // finally null

        if (currentObject.GetComponent<Rigidbody>())
        {
            activeMesh = currentObject;
        }
        else if (currentObject.transform.parent.gameObject.GetComponent<Rigidbody>())
        {
            activeMesh = currentObject.transform.parent.gameObject;
        }
        else if (currentObject.GetComponentInChildren<Rigidbody>())
        {
            activeMesh = currentObject.GetComponentInChildren<Rigidbody>().gameObject;
        }
        else if (currentObject.GetComponent<Collider>())
        {
            activeMesh = currentObject;
        }
        else if (currentObject.transform.parent.gameObject.GetComponent<Collider>())
        {
            activeMesh = currentObject.transform.parent.gameObject;
        }
        else if (currentObject.GetComponentInChildren<Collider>())
        {
            activeMesh = currentObject.GetComponentInChildren<Collider>().gameObject;
        }
        else
        {
            activeMesh = null;
        }
    }

    private Vector3 GetDraggingPoint()
    {
        if (isGameMode)
        {
            return Camera.main.ViewportToWorldPoint(new Vector3(0.5f, 0.5f, screenPoint.z)) + offset;
        }
        else
        {
            Vector3 curScreenPoint = new Vector3(Input.mousePosition.x, Input.mousePosition.y, screenPoint.z);
            Vector3 objPos = Camera.main.ScreenToWorldPoint(curScreenPoint) + offset;
            return objPos;
            //float distance_to_screen = Camera.main.WorldToScreenPoint(gameObject.transform.position).z;
            //Vector3 pos_move = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, distance_to_screen));
            //return new Vector3(pos_move.x, transform.position.y, pos_move.z);
        }
    }

    /*
    void DisableEverything()
    {
        translationEnabled = false;
        rotationEnabled = false;
    }

    /// Enables rotation and translation control for desktop environments.
    /// For mobile environments, it enables rotation or translation according to
    /// the device capabilities.
    void EnableAccordingToPlatform()
    {
        rotationEnabled = inDesktopLike || !capabilities.canPresent;
        translationEnabled = inDesktopLike || !capabilities.hasPosition;
    }
    */

    public static float ClampAngle (float angle, float min, float max)
    {
        if (angle < -360f)
            angle += 360f;
        if (angle > 360f)
            angle -= 360f;
        return Mathf.Clamp (angle, min, max);
    }

    IEnumerator RotateCamera(Quaternion targetRot, float speed)
    {
        float rotationTimer = 0.0f;

        while (rotationTimer < 0.8)
        {
            rotationTimer += Time.smoothDeltaTime * 1f;
            transform.localRotation = Quaternion.Slerp(transform.localRotation, targetRot, rotationTimer * speed);
            yield return new WaitForEndOfFrame();
        }
    }

    IEnumerator TranslateCamera(Vector3 targetPos, float speed)
    {
        float strafeTimer = 0.0f;
        
        while (strafeTimer < 0.8)
        {
            strafeTimer += Time.smoothDeltaTime * 1f;
            transform.localPosition = Vector3.Lerp(transform.localPosition, targetPos, strafeTimer * speed);
            yield return new WaitForEndOfFrame();
        }
    }

    

    private void SingleClick()
    {
        if (activeMesh != null)
        {
            string prompt = "";
            switch (activeMesh.name)
            {
                
                case "InfoPoint":
                    prompt = "Double click to \n show next slide";
                    break;

                case "Projector":
                    prompt = "Double click to \n play video";
                    break;
                default:
                    prompt = activeMesh.name;
                    break;
            }

            if (!isHudBusy && LabelObject != null)
            {
                StartCoroutine(FloatUpAndOut(activeMesh, prompt));
            }
        }

    }



    private void RightClick()
    {
        activeMesh.SendMessage("OnRightClick", activeMesh);
    }


    private void DoubleLeftClick()
    {
        if (activeMesh != null)
        {
            Debug.Log("Interaction with " + activeMesh.name);
            activeMesh.SendMessage("OnDoubleClick", activeMesh);

            if (activeMesh.name == "Exit_Object")
            {
                openWindow(exitUrl);
            }
        }
    }

    IEnumerator FloatUpAndOut(GameObject focusedObj, string _text, float speed = 0.5f)
    { 
        isHudBusy = true;
        
        Vector3 startPos = focusedObj.transform.position;
        Quaternion lookAtMe = Quaternion.LookRotation(-(Camera.main.transform.position - focusedObj.transform.position),Vector3.up);
        
        GameObject newLabel = Instantiate(LabelObject, startPos, lookAtMe, Camera.main.transform);
        newLabel.transform.localScale = Vector3.one / focusedObj.transform.localScale.magnitude;    
        newLabel.transform.position = startPos;

        Vector3 targetPos = startPos + (Vector3.up * 2.0f);
        TextMesh myTM = newLabel.GetComponent<TextMesh>();
        myTM.text = _text;

        float floatTimer = 0.0f;

        while (floatTimer < 1.0f)
        {
            floatTimer += Time.smoothDeltaTime * 1f;
            newLabel.transform.position = Vector3.Lerp(startPos, targetPos, floatTimer * speed);
            yield return new WaitForEndOfFrame();
        }

        Destroy(newLabel, 1.5f);
        isHudBusy = false;
    }


    private GameObject ScreenRaycast()
    {
        if (isDragging) return currentObject;

        RaycastHit pointerHit = new RaycastHit();
        Ray ray;
        if (isGameMode)
        {
            int x = (Screen.width / 2);
            int y = (Screen.height / 2);
            ray = Camera.main.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));
        }
        else
        {
            ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        }

        if (Physics.Raycast(ray, out pointerHit, 15.0f, Physics.DefaultRaycastLayers))
        {
            currentDistance = pointerHit.distance;
            return pointerHit.transform.gameObject;
        }
        else
        {

            return null;
        }
    }
}
