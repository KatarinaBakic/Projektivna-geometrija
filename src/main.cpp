#include <iostream>
#include <GL/glut.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::MatrixXf;



MatrixXf p1(1, 3), p2(1, 3), q1(1, 3), q2(1, 3);
static int window_width, window_height;


static int timer_active = 0 ;
static float timer = 0; 
double parametar = 0, direction = 1;
double parametar_max, parametar_min, s1, s2, s3 ;

void nacrtaj_ose();
static void on_keyboard(unsigned char key, int x, int y);
static void on_timer(int value);
static void on_reshape(int width, int height);
static void  on_display(void);
void pocetni_krajnji_parametri();
void svetlo_materijali();


MatrixXf Euler2A( double ugao1, double ugao2, double ugao3 );
void AxisAngle( MatrixXf A, MatrixXf &p, double & fi );
MatrixXf Rodigez( MatrixXf P, double fi );
MatrixXf A2Euler( MatrixXf A );
MatrixXf AxisAngle2Q( MatrixXf p, double fi );
void Q2AxisAngle( MatrixXf q, MatrixXf &p, double & fi );
void ortogonalna( MatrixXf A ); 
Eigen::Vector4d slerp( MatrixXf q1, MatrixXf q2, double t_m, double par );

int main (int argc, char ** argv) {
 
   
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
 	glutInitWindowSize(1000, 1000);
	glutCreateWindow("Animacija: ");
	    
    glClearColor(0, 0, 0, 1);
    
    svetlo_materijali();
    
    glutReshapeFunc(on_reshape);
    glutKeyboardFunc(on_keyboard);
    glutDisplayFunc(on_display);
    glutIdleFunc(on_display);

//     pocetni_krajnji_parametri();
    
      
    glutMainLoop();
	return 0;
}




static void on_keyboard(unsigned char key, int x, int y){
    switch (key) {
        case 27:
            /* Zavrsava se program. */
            exit(0);
            break;
        
        case 'g' :
        case 'G' :
            if (!timer_active) {
                glutTimerFunc(50, on_timer, 0);
                timer_active = 1;
            }
            break;
        
        case 's' :
        case 'S' :
            timer_active = 0;
            break;
    }
}

static void on_reshape(int width, int height) {
    /* Pamte se sirina i visina prozora. */
    window_width = width;
    window_height = height;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective( 40, window_width/(float)window_height, 1, 500. );

    
}

void pocetni_krajnji_parametri(){
    // Pocetni polozaj: 
   
    p1 << 3.2, 3.4, 4.2 ;
    
    MatrixXf vektor_p1(1, 3), vektor_p2(1, 3);
    double fi1, fi2 ;
    AxisAngle(Euler2A(3*M_PI/4.0, M_PI/4.0, 2*M_PI/5.0), vektor_p1, fi1);
    
    //std::cout<< "Pocetna pozicija: "<< p1<< "\nVektor_p: "<<vektor_p<<"\n ugao FI: "<<fi << std::endl ;
    
    q1 = AxisAngle2Q (vektor_p1, fi1);
    //std::cout<< "Q1 : "<<q1 <<"\n";
    
    // Krajnji polozaj: 
    p2 << -0.25, -0.45, -1.9 ;
    
    AxisAngle( Euler2A( 2*M_PI/3.0, -4*M_PI/7.0, 7*M_PI/4.0 ), vektor_p2, fi2 );
    
    q2 = AxisAngle2Q( vektor_p2, fi2 );
    //std::cout<< "Q2 : "<<q2 <<"\n";
    
    parametar_max = 15;
    parametar_min = 0;
    
}

void nacrtaj_ose(){
    
  glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(7,0,0);
    glEnd();
    
    glBegin(GL_LINES);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,7,0);
    glEnd();
    
    glBegin(GL_LINES);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,7);
    glEnd();
      
}
void svetlo_materijali(){
    
    glEnable(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    /*Koeficijenti za osvetljenje: pozicija svetla, ambientalno, difuzno i spekularno */
    GLfloat light_position[] = {5, 5, 5, 2};
	GLfloat light_diffuse[]  = {1, 1, 1, 1};
	    
    /*Ukljucivanje svetla: */
        
    /*Postavljanje svetla: */
    glLightfv( GL_LIGHT0, GL_POSITION, light_position   );
    glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_diffuse    );
    glLightf ( GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.01 );
    glLightf ( GL_LIGHT0, GL_LINEAR_ATTENUATION,   0.01 );
	
    glEnable(GL_LIGHT0);
}

static void on_timer(int value) {
    
    if(value != 0)
        return;
    
    timer += 0.2;
    std::cout<<timer<<"\n";
    if( timer > parametar_max ) {
        timer = 0;
        timer_active = 0;
        return;
    }
        
    glutPostRedisplay();
    
    if(timer_active)
        glutTimerFunc(50, on_timer, 0);
}


static void on_display(void){
    
    /* Brise se prethodni sadrzaj prozora. */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    
    /* Podesava se tacka pogleda. */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt( 15, 20, 15,
               0, 0, 0,
               0, 1, 0
              );

    /* Zelimo da objekti budu zadate boje */
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    
    nacrtaj_ose();
    pocetni_krajnji_parametri();
    
    
    glPushMatrix();
    
    
    MatrixXf  pk(1, 3);
    
    pk = (1 - parametar/parametar_max )*p1 + ( parametar/parametar_max )*p2 ;

    double x = pk(0), y = pk(1), z = pk(2) ;    
    glTranslatef(x, y, z);
    
    
    Eigen::Vector4d qs = slerp( q1, q2, parametar_max, timer );
    
    MatrixXf qs_matrix (1, 4), vektor_p(1, 3), vektor_p1(1, 3), vektor_p2(1, 3) ;
    
    // potreban nam je qs kao matrica, radi daljeg racuna
    qs_matrix << qs(0), qs(1), qs(2), qs(3);
    //std::cout<<"QS : "<< qs_matrix<<"\n";
    
    double fi ;
    
    
    Q2AxisAngle ( qs_matrix, vektor_p, fi);


    glRotatef(fi/M_PI*180, vektor_p(0), vektor_p(1), vektor_p(2) );
        
    
    
    glColor3f(1, 0.4, 0);
    glutSolidCube(2);
    nacrtaj_ose();
    
    glPopMatrix();
    
    
    glutSwapBuffers();

}


MatrixXf Euler2A(double ugao1, double ugao2, double ugao3){

//     std::cout << "\nUlazni uglovi:\n" << "fi :" <<ugao1 << ", teta:" << ugao2 << ", psi: "<<ugao3 << "\n\n" ;
    MatrixXf Rx(3, 3), Ry(3, 3), Rz(3, 3);
    Rx<< 1, 0, 0,
         0, cos(ugao1), -sin(ugao1),
         0, sin(ugao1), cos(ugao1);
        
    Ry<< cos(ugao2), 0, sin(ugao2),
         0, 1, 0,
        -sin(ugao2), 0, cos(ugao2);

    Rz<< cos(ugao3), -sin(ugao3), 0,
         sin(ugao3), cos(ugao3), 0,
         0, 0, 1;
         
    return Rz * Ry * Rx;  
}

/*ulaz: ortogonalna matrica A , detA= 1
  izlaz: jedinicni vektor p i ugao fi, takav da je A= Rp(fi)
*/
void AxisAngle ( MatrixXf A, MatrixXf &p, double & fi ){
    
    MatrixXf E(3, 3);
    E<< 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
       
    //moramo da proverimo da li je matrica A ortogonalna:
    // ortogonalna(A);  
       
    /*prvo odredjujemo sopstveni vektor p  za lambda=1;   
      p je resenje sistema A-lambda*E= A-E, 
      tj. (A-lambda*E)p=0 , p= x,y,z;
    */
    MatrixXf P(3, 3);
    P= A - E;
    // std::cout<<"P:\n "<<P<<std::endl;
    
    //sistem je ranga 2 ili manje, pa su nam dovoljne samo prve 2 jednacine:   
    Eigen::Vector3d v1(P(0, 0), P(0, 1), P(0, 2));
    Eigen::Vector3d v2(P(1, 0), P(1, 1), P(1, 2));
    Eigen::Vector3d v3(P(2, 0), P(2, 1), P(2, 2));
    
    //p = v1 X v2;
    Eigen::Vector3d p_cross= v1.cross(v2);
    
    
    // std::cout<<" v1 i v2: \n"<< p_cross<<"\n";
    
    
    //treba nam norma da odredimo jedinicni vektor :
    double norma_p= sqrt(p_cross(0) * p_cross(0) + p_cross(1) * p_cross(1) + p_cross(2) * p_cross(2));
    
    if(norma_p==0){
        p_cross=v1.cross(v3);
        norma_p= sqrt(p_cross(0) * p_cross(0) + p_cross(1) * p_cross(1) + p_cross(2) * p_cross(2));
    
    }
    
//       std::cout<<"norma_p :\n"<<norma_p<<"\n";
    
    Eigen::Vector3d p_jedinicni= ( 1 / norma_p) * p_cross;
    // std::cout<<p_jedinicni<<"\n";
    
    
    // treba da odredimo vektor koji je normalan na p:  to je npr vektor v1 ili v2:
    Eigen::Vector3d u= v2;
    // std::cout<< "u:\n"<<u<<"\n";
    
    // radi lakseg mnozenja prebacimo u matricu: 

    MatrixXf u_pom (3, 1);
    u_pom<< u(0), u(1), u(2);
    // std::cout<<u_pom<<"\n";

    MatrixXf u_primPom= A * u_pom;

    //vratimo rezulat u vektor za dalji racun:
    Eigen::Vector3d u_prim (u_primPom(0), u_primPom(1), u_primPom(2));
    //   std::cout<<"u_prim:\n"<<u_prim<<"\n";
    
    // trazimo norme za u i uPrim:
     double norma_u= sqrt(u(0) * u(0) + u(1) * u(1) + u(2) * u(2));
     double norma_uPrim= sqrt(u_prim(0) * u_prim(0) + u_prim(1) * u_prim(1) + u_prim(2) * u_prim(2));
    
    /*--trazimo ugao fi= arcsoc((u*uPrim)/(norma_u*norma_uPrim)):---*/
    /* proizvod 2 vektora: (x,y,z)*(x1,y1,z1)= x*x1+y*y1+z*z1: ---> koristimo dot: */
    
    double proizvod_vektori= u.dot (u_prim);
    double proizvod_norme= norma_u * norma_uPrim;
    //dobijamo ugao fi:
    fi= acos(proizvod_vektori / proizvod_norme);
    
    
    /*proveravamo da li je determinanta proizvoda [u,uPrim,p] <0, ako jeste onda je p=-p:
         da bi rotacija bila u pozitivnom smeru */
    MatrixXf proizvod(3, 3);
    proizvod<< u(0), u(1), u(2),
               u_prim(0), u_prim(1), u_prim(2),
               p_jedinicni(0), p_jedinicni(1), p_jedinicni(2);
    // std::cout<<"proizvod :\n"<<proizvod<<"\n\n";
    
    double det= proizvod.determinant();
    // std::cout<<"det: \n"<<det<<"\n";
    
    //da bi rotacija bila u pozitivnom smeru:
    if(det < 0){
     p_jedinicni= p_jedinicni*(-1);   
    }
    
    //vracamo jedinicni vektor p:
    p<< p_jedinicni(0), p_jedinicni(1), p_jedinicni(2);
    
}

/*vracamo matricu rotacije oko orijentisanog (jedinicnog) vektora p, za ugao fi: */

MatrixXf Rodigez(MatrixXf P, double fi){
        
    MatrixXf E(3, 3), px(3, 3);
    E<< 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
        
    //matrica vektorskog mnozenja jedinicnim vektorom p=(p1,p2,p3)
        
    px<< 0, -P(2), P(1),
        P(2), 0, -P(0),
        -P(1), P(0), 0;
    
    //std::cout<<"px :\n "<<px<<"\n\n";    
   
    
    //potrebna nam je matrica Rp(φ) =ppT+ cosφ(E−ppT) + sinφp×,

    //mnozimo P i P transponovano:
    MatrixXf Ppom(3, 1);
    Ppom<< P(0), P(1), P(2);
    
    MatrixXf Ptrans= Ppom.transpose();
    
    MatrixXf PPtrans= Ppom * Ptrans;
    //std::cout<<"P :\n"<<PPtrans<<"\n\n";
    
    //cos(fi)*(E-P*Ptrans); -- drugi sabirak u Rp(fi):
    MatrixXf CosFi= cos(fi) * (E - PPtrans);
    //std::cout<< "cos(fi):\n"<< CosFi<< "\n\n";
    
    //sin(fi)*px --treci sabirak u Rp(fi):
    MatrixXf SinFi= sin(fi) * px;
    //std::cout<< "sin(fi):\n"<< SinFi<< "\n\n";
    
    MatrixXf Rp= PPtrans + CosFi + SinFi;

    return Rp;    
    
}

/* 
  ulaz: ortogonalna matrica A , detA= 1; 
  izlaz:  Ojlerovi uglovi: */
MatrixXf A2Euler (MatrixXf A){
    
    double ugao1, ugao2, ugao3;
    
    if(A(2, 0) < 1){
        
        if(A(2, 0) > -1){
        
            ugao1= atan2(A(2, 1), A(2, 2));
            ugao2= asin(-A(2, 0));
            ugao3= atan2(A(1, 0), A(0, 0));
        }
        else{
        
            ugao1= 0;
            ugao2= M_PI/2;
            ugao3= atan2(-A(0, 1), A(1, 1));
        }        
    }
    else{
    
        ugao1= 0;
        ugao2= -M_PI/2;
        ugao3= atan2(-A(0, 1), A(1, 1));
    }
    
    MatrixXf rez(1, 3);
    rez<< ugao1, ugao2, ugao3;
    
    return rez;
}

/* rotacija u kvaternionu:
   ulaz: rotacija oko ose p za ugao fi
   izlaz: jedinicni kvaternion, q=[x,y,z,w]=[sin(fi)*(px, py, pz),cos(fi)]; */
MatrixXf AxisAngle2Q (MatrixXf p, double fi){
    
    double w= cos(fi/2);
    p= p.normalized();
    
    double x= sin(fi/2) * p(0);
    double y= sin(fi/2) * p(1);
    double z= sin(fi/2) * p(2);
    
    MatrixXf q(1, 4);
    q<< x, y, z, w;
    
    return q;
    
}
/*kvaternion u rotaciji:
 ulaz: jedinicni kvaternion q 
 izlaz: jedinicni vektor p i ugao fi=[0, 2*pi): */
void Q2AxisAngle(MatrixXf q, MatrixXf &p, double & fi){
   
    q= q.normalized();
    double w= q(3);
    
    if(w < 0){
     q= q*(-1);   
    }
    fi = 2.0 * acos(w);

    p<< q(0), q(1), q(2);
    if( abs(w) == 1 ) {
        p<< 1, 0, 0;   
    }
    else {
        p = p.normalized();   
    }
    //std::cout<<p<<"\n";
}

void ortogonalna(MatrixXf A){
    
//      double det = A.determinant();
//      std::cout<<det<<"\n";
    MatrixXf E(3, 3);
    E<< 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    MatrixXf A_pom= A.transpose()*A;
    
    if (A_pom.isApprox(E)){
        std::cout<< "( Jeste ortogonalna matrica. )\n";   
    }
    else{           
        std::cout<< "( Nije ortogonalna matrica. )\n";   
    }
}


/* funkcija SLerp:
    ulaz : 2 jedinicna kvarterniona q1 i q2, duzinu interpolacije t_m i parametar t=[0,t_m]
    izlaz: jedinicni kvarternion qs(t), koji zadaje orijentaciju u trenutku t.
 */
Eigen::Vector4d slerp (MatrixXf q1, MatrixXf q2, double t_m, double par){
    
    //cos0=<q1,q2>
    
     // treba nam vektor, zbog funkcije .dot ;
     Eigen::Vector4d q1_vec (q1(0), q1(1), q1(2), q1(3));     
     Eigen::Vector4d q2_vec (q2(0), q2(1), q2(2), q2(3));
     
     double q1_norma = q1_vec.norm();
     double q2_norma = q2_vec.norm();
          
     if(par == 0)
       return q1_vec;   
     
     if( par == t_m)
         return q1_vec;
     
     //norme su 1:
     
     double sk_proizvod= q1_vec.dot(q2_vec)/(q1_norma * q2_norma);
     
     //std::cout<<"Skalarni proizvod: "<< sk_proizvod <<"\n";
    
     //idi po kracem luku sfere:
     
     if (sk_proizvod < 0){
          q1= q1*(-1);
          sk_proizvod= sk_proizvod*(-1);
     }
    
    // ako su kvarternioni previse blizu:
     if (sk_proizvod > 0.95){
         return q1_vec;
     }
    
    //trazimo ugao: fi = acos (cos0<q1, q2>);
    
    double fi= acos(sk_proizvod);
    
    Eigen::Vector4d qs = ( sin (fi*(1 - par/ t_m )))/ ( sin(fi))* q1_vec + (sin(fi* par/ t_m))/ (sin(fi))* q2_vec;
    
    return qs;
    
}







