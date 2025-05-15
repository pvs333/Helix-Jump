using UnityEngine;

public class Disc : MonoBehaviour
{
    public int rotSpd;
    GameObject ball;
    GameObject GM;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ball = GameObject.FindGameObjectWithTag("Player");
        GM = GameObject.FindGameObjectWithTag("GameManager");
    }

    // Update is called once per frame
    void Update()
    {
        if(ball != null){
        if(Vector3.Distance(ball.transform.position, transform.position) > 20){
            Destroy(gameObject);
        }
        }
        if(Input.GetKey(KeyCode.A)){
            transform.Rotate(Vector3.up, 1f);
        }else if(Input.GetKey(KeyCode.D)){
            transform.Rotate(Vector3.up, -1f);
        }
        if(Input.GetMouseButton(0)){
            float X = Input.GetAxisRaw("Mouse X");
            transform.Rotate(transform.position.x, -X*rotSpd*Time.deltaTime, transform.position.z);
        }
        
    }
}
