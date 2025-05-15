using TMPro;
using Unity.VisualScripting;
using UnityEngine;

public class Ball : MonoBehaviour
{
    public Rigidbody rb;
    public int jump;
    public GameObject blast, dS;
    public GameManager GM;
    
    
    int y;

    void OnCollisionEnter(Collision other)
    {
        if(other.gameObject.GetComponent<MeshRenderer>().material.name[0] != 'D'){
            rb.linearVelocity = Vector3.up * jump;
        }else{
            Camera.main.transform.parent = null;

            Instantiate(blast,transform.position, Quaternion.identity);
            dS.SetActive(true);
            Destroy(gameObject);
            
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if(y != Mathf.RoundToInt(other.gameObject.transform.position.y))
        GM.S++;
        Debug.Log("SCORE: "+GM.S);
        y = Mathf.RoundToInt(other.gameObject.transform.position.y);
        
    }
}
