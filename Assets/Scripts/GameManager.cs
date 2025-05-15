using UnityEngine;
using TMPro;
using UnityEngine.SceneManagement;

public class GameManager : MonoBehaviour
{
    public GameObject Ball;
    public GameObject Disc;
    public GameObject LastDisc, nHS;
    public int S, HS;
    public TMP_Text score, fS, fHS;

    void Start()
    {
        LastDisc = Instantiate(Disc, transform.position, Quaternion.identity);
        HS = PlayerPrefs.GetInt("HS");
    }

    // Update is called once per frame
    void Update()
    {
        if(Ball != null)
        if(Vector3.Distance(Ball.transform.position, LastDisc.transform.position) < 5){
            LastDisc = Instantiate(Disc, LastDisc.transform.position - new Vector3(0, 3, 0), Quaternion.identity);
        }
        score.SetText(S+"");
        fS.SetText("Score - " + S);
        fHS.SetText("" + HS);
        
        if(S>HS){
            HS = S;
            PlayerPrefs.SetInt("HS", HS);
            Debug.Log("HIGHSCORE: "+HS);
            nHS.SetActive(true);
        }

        if(Input.GetKeyDown(KeyCode.R)){
            Restart();
        }
    }

    public void Restart(){
        SceneManager.LoadScene(0);
    }
}
