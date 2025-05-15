using UnityEngine;

public class DiscLogic : MonoBehaviour
{
    public GameObject[] DiscPieces;
    public GameObject Cylinder;
    public Material good, bad;

    void Start(){
        int pass = Random.Range(0, DiscPieces.Length);
        int pass2 = 0;
        if(pass == 7) pass2 = 0;
        else pass2 = pass +1;
        int critical = Random.Range(0, DiscPieces.Length);
        while(pass == critical || pass2 == critical){
            critical = Random.Range(0, DiscPieces.Length);
        }
        float dis = Random.Range(-0.8f,0.8f);
        transform.position = new Vector3(transform.position.x, transform.position.y + dis, transform.position.z);
        DiscPieces[pass].GetComponent<MeshRenderer>().enabled = false;
        DiscPieces[pass2].GetComponent<MeshRenderer>().enabled = false;
        DiscPieces[pass].GetComponent<MeshCollider>().isTrigger = true;
        DiscPieces[pass2].GetComponent<MeshCollider>().isTrigger = true;
        for(int i = 0; i<8 ;i++){
            DiscPieces[i].GetComponent<MeshRenderer>().material = good;
        }
        DiscPieces[critical].GetComponent<MeshRenderer>().material = bad;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
