using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

public class ShowTheWorld : MonoBehaviour {

    public GameObject MenuItemPrefab;

    [HideInInspector]
    public ulong[] MenuKeys;
    public string[] MenuNames;

    private bool isMenuOpen = false;
    private float menuItemDistance = 1.5f; // radial distance from object
    private float menuSpeed = 7f; // rate of menu expansion/contraction


    private void Start()
    {

    }

    private void Update()
    {


    }

    public void OnRightClick()
    {
        Toggle_Menu();
    }


    public void Toggle_Menu()
    {
        if (isMenuOpen) { CloseMenu(); }
        else { OpenMenu(MenuKeys, MenuNames); }
    }


    private void OpenMenu(ulong[] keys, string[] names)
    {
        if (keys.Length == 0)
        {
            Debug.Log("no menu items found");
            return;
        }

        isMenuOpen = true;

        Vector3 startingPoint = transform.position;
        float menuThetaOffset = 6.283f / (keys.Length * 2);
        if (keys.Length > 10) { menuThetaOffset = 6.283f / keys.Length; }

        GameObject[] newMenu = new GameObject[keys.Length];

        for (int i = 0; i < keys.Length; i++)
        {
            newMenu[i] = Instantiate(MenuItemPrefab);
            newMenu[i].name = names[i];

            var myBCH = newMenu[i].GetComponent<BubbleClickHandler>();
            myBCH.DeviceAddress = keys[i];
            myBCH.rootNode = transform.gameObject;

            newMenu[i].transform.SetParent(transform);
            newMenu[i].SetActive(true);
            newMenu[i].transform.rotation = Quaternion.LookRotation(transform.position - Camera.main.transform.position);
            newMenu[i].transform.position = startingPoint;

            var textDisplay = newMenu[i].GetComponentInChildren<TextMesh>();
            textDisplay.text = names[i].ToString();

            // arrange items evenly in a semi-circle (in radians)
            Vector2 itemPos;
            float newMID = menuItemDistance * (transform.localScale.magnitude);
            itemPos.x = newMID * Mathf.Cos(menuThetaOffset * i);
            itemPos.y = newMID * Mathf.Sin(menuThetaOffset * i);
            Vector3 endPos = (itemPos.x * Camera.main.transform.right) + (itemPos.y * Camera.main.transform.up);

            StartCoroutine(Translation(newMenu[i].gameObject, startingPoint, startingPoint + endPos, menuSpeed));
        }
    }

    private void CloseMenu()
    {
        for (int y = 0; y < transform.childCount; y++)
        {
            if (transform.GetChild(y).gameObject.GetComponent<BubbleClickHandler>())
            {
                Destroy(transform.GetChild(y).gameObject);
            }
        }
        isMenuOpen = false;
    }



    private IEnumerator Translation(GameObject thisGameObject, Vector3 startPos, Vector3 endPos, float speed)
    {
        float rate = 1.0f / Vector3.Distance(startPos, endPos) * speed;
        float t = 0.0f;
        while (t < 1.0)
        {
            t += Time.deltaTime * rate;
            if (thisGameObject != null)
            {
                thisGameObject.transform.position = Vector3.Lerp(startPos, endPos, Mathf.SmoothStep(0.0f, 1.0f, t));
            }
            yield return null;
        }
    }


}
