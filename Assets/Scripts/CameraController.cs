using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    // Start is called before the first frame update

    private float lookSpeed, mouseX, mouseY;
    public float moveSpeed;
    private AgentManager am;

    void Start()
    {
        lookSpeed = 7.0f;
        mouseX = mouseY = 0.0f;
        moveSpeed = 10;
        am = GameObject.FindGameObjectWithTag("Manager").GetComponent<AgentManager>();
    }

    // Update is called once per frame
    void Update()
    {
        Movement();
        DetectObjects();
    }
    // Camera free-look, moves where camera is pointing
    private void Movement()
    {

        mouseX = (Input.mousePosition.x / Screen.width) - 0.5f;
        mouseY = (Input.mousePosition.y / Screen.height) - 0.5f;
        transform.localRotation = Quaternion.Euler(new Vector4(-1f * (mouseY * 180f), mouseX * 360f, transform.localRotation.z));

        Vector3 input = Quaternion.Euler(0, transform.eulerAngles.y, 0) * new Vector3(Input.GetAxis("Horizontal"), 0.0f, Input.GetAxis("Vertical"));


        transform.position += input * Time.deltaTime * moveSpeed;

        if (Input.GetKey(KeyCode.Space))
        {
            transform.position += Vector3.up * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.LeftShift))
        {
            // Only if we are above ground
            if (transform.position.y > 0.0f)
            {
                transform.position -= Vector3.up * Time.deltaTime * moveSpeed;
            }
        }
    }
    private void DetectObjects()
    {
        if (Input.GetMouseButtonDown(0))
        {

            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, 1500))
            {
                if (hit.transform)
                {
                    Debug.DrawRay(hit.point, Vector3.up, Color.red, 5f);
                    am.SetAgentDestinations(hit.point);

                }

            }

        }
    }
}
