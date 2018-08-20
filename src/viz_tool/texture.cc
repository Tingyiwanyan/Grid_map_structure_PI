
#include "viz_tool/texture.h"

namespace viz_tool
{


void readstr(FILE *f, char *string)
{
    do {
        if(fgets(string, 255, f) ==NULL) { printf("Failed reading files!\n"); } // read the line
    } while ((string[0] == '/') || (string[0] == '\n'));
    return;
}


// loads the world from a text file.
void loadConfigs(SECTOR& sector, char* filename) 
{
    float x, y, z, u, v;
    int vert;
    int numtriangles;
    FILE *filein;        // file to load the world from
    char oneline[255];

    filein = fopen(filename, "rt");

    readstr(filein, oneline);
    sscanf(oneline, "NUMPOLLIES %d\n", &numtriangles);

    sector.numtriangles = numtriangles;
    sector.triangle = (TRIANGLE *) malloc(sizeof(TRIANGLE)*numtriangles);
    
    for (GLuint loop = 0; loop < (GLuint)numtriangles; loop++) {
	for (vert = 0; vert < 3; vert++) {
	    readstr(filein,oneline);
	    sscanf(oneline, "%f %f %f %f %f", &x, &y, &z, &u, &v);
	    sector.triangle[loop].vertex[vert].x = x;
	    sector.triangle[loop].vertex[vert].y = y;
	    sector.triangle[loop].vertex[vert].z = z;
	    sector.triangle[loop].vertex[vert].u = u;
	    sector.triangle[loop].vertex[vert].v = v;
	}
    }

    fclose(filein);
    return;
}
 

// Load Bitmaps And Convert To Textures
GLvoid loadGLTextures(GLuint texture[][3]) 
{	
    // Load Texture
    Image *image1;
    
    // allocate space for texture
    image1 = (Image *) malloc(sizeof(Image));
    if (image1 == NULL) { 
	printf("Error allocating space for image");
	exit(0);
    }

    if (!ImageLoad((char*)"textures/grey_bricks.bmp", image1)) {
    //if (!ImageLoad((char*)"textures/dark_bricks.bmp", image1)) {
	std::cerr<<"Failed loading brick wall texture!"<<std::endl;
	exit(1);
    }        

    // Create Textures	
    glGenTextures(3, &texture[0][0]);

    // nearest filtered texture
    glBindTexture(GL_TEXTURE_2D, texture[0][0]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST); // scale cheaply when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST); // scale cheaply when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    // linear filtered texture
    glBindTexture(GL_TEXTURE_2D, texture[0][1]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    // mipmapped texture
    glBindTexture(GL_TEXTURE_2D, texture[0][2]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST); // scale mipmap when image smalled than texture
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image1->sizeX, image1->sizeY, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    if (!ImageLoad((char*)"textures/crate.bmp", image1)) {
	std::cerr<<"Failed loading crate texture!"<<std::endl;
	exit(1);
    }        
    // Create Textures	
    glGenTextures(3, &texture[1][0]);

    // nearest filtered texture
    glBindTexture(GL_TEXTURE_2D, texture[1][0]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST); // scale cheaply when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST); // scale cheaply when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    // linear filtered texture
    glBindTexture(GL_TEXTURE_2D, texture[1][1]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    // mipmapped texture
    glBindTexture(GL_TEXTURE_2D, texture[1][2]);   // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST); // scale mipmap when image smalled than texture
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image1->sizeX, image1->sizeY, GL_RGB, GL_UNSIGNED_BYTE, image1->data);

    free(image1);

};


// build the display list for a cube, e.g., the crate box.
GLvoid cubeList(GLuint& cube) 
{

    cube = glGenLists(1);              // generate storage for 2 lists, and return a pointer to the first.
    //cout<<"List cube: "<<cube<<endl;
    glNewList(cube, GL_COMPILE);       // store this list at location cube, and compile it once.
    glNormal3f( 0.0f, 1.0f, 0.0f);

    GLfloat cube_width = 3.0/2;		//half cube width
    Vec3f normal;
    //glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); // for wrong normal

    glBegin(GL_QUADS);			

    // Bottom Face
    // must set the normal before defining the plane
    // remember the z direction is pointing out of paper
    normal = computeSurfaceNormal(
		Vec3f(-cube_width, -cube_width, -cube_width), 
		Vec3f(cube_width, -cube_width,  -cube_width), 
		Vec3f(cube_width, -cube_width,  cube_width) 
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f(-cube_width, -cube_width, -cube_width);	// Top Right 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f( cube_width, -cube_width, -cube_width);	// Top Left 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f( cube_width, -cube_width,  cube_width);	// Bottom Left 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f(-cube_width, -cube_width,  cube_width);	// Bottom Right 
    
    // Front Face
    normal = computeSurfaceNormal(
		Vec3f(-cube_width, -cube_width, cube_width), 
		Vec3f(cube_width, -cube_width,  cube_width), 
		Vec3f(cube_width, cube_width,  cube_width)
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f(-cube_width, -cube_width,  cube_width);	// Bottom Left 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f( cube_width, -cube_width,  cube_width);	// Bottom Right 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f( cube_width,  cube_width,  cube_width);	// Top Right 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f(-cube_width,  cube_width,  cube_width);	// Top Left 
    
    // Back Face
    normal = computeSurfaceNormal(
		Vec3f(-cube_width, -cube_width, -cube_width), 
		Vec3f(-cube_width, cube_width,  -cube_width), 
		Vec3f(cube_width, cube_width,  -cube_width) 
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f(-cube_width, -cube_width, -cube_width);	// Bottom Right 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f(-cube_width,  cube_width, -cube_width);	// Top Right 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f( cube_width,  cube_width, -cube_width);	// Top Left 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f( cube_width, -cube_width, -cube_width);	// Bottom Left 
    
    // Right face
    normal = computeSurfaceNormal(
		Vec3f(cube_width, -cube_width, -cube_width), 
		Vec3f(cube_width, cube_width,  -cube_width), 
		Vec3f(cube_width, cube_width,  cube_width) 
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f( cube_width, -cube_width, -cube_width);	// Bottom Right 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f( cube_width,  cube_width, -cube_width);	// Top Right 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f( cube_width,  cube_width,  cube_width);	// Top Left 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f( cube_width, -cube_width,  cube_width);	// Bottom Left 
    
    // Left Face
    normal = computeSurfaceNormal(
		Vec3f(-cube_width, -cube_width, -cube_width), 
		Vec3f(-cube_width, -cube_width,  cube_width), 
		Vec3f(-cube_width, cube_width,  cube_width) 
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f(-cube_width, -cube_width, -cube_width);	// Bottom Left 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f(-cube_width, -cube_width,  cube_width);	// Bottom Right 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f(-cube_width,  cube_width,  cube_width);	// Top Right 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f(-cube_width,  cube_width, -cube_width);	// Top Left 

    // Top Face
    normal = computeSurfaceNormal(
		Vec3f(-cube_width, cube_width, -cube_width), 
		Vec3f(-cube_width, cube_width,  cube_width), 
		Vec3f(cube_width, cube_width,  cube_width) 
		);
    glNormal3f(normal.x(), normal.y(), normal.z());	// set normal of plane
    glTexCoord2f(0.0f, 0.0f); 
    glVertex3f(-cube_width,  cube_width, -cube_width);	// Top Left 
    glTexCoord2f(0.0f, 1.0f); 
    glVertex3f(-cube_width,  cube_width,  cube_width);	// Bottom Left 
    glTexCoord2f(1.0f, 1.0f); 
    glVertex3f( cube_width,  cube_width,  cube_width);	// Bottom Right 
    glTexCoord2f(1.0f, 0.0f); 
    glVertex3f( cube_width,  cube_width, -cube_width);	// Top Right 

    glEnd();

    glEndList();
}


//
GLvoid planeList(GLuint& plane, SECTOR& sec)
{
    GLfloat x_m, y_m, z_m, u_m, v_m;
    int numtriangles;

    plane = glGenLists(1);              // generate storage for 2 lists, and return a pointer to the first.
    //cout<<"List plane: "<<plane<<endl;
    glNewList(plane, GL_COMPILE);       // store this list at location cube, and compile it once.
    glNormal3f( 0.0f, 1.0f, 0.0f);

    //draw walls, eg.
    numtriangles = sec.numtriangles;
    for (GLuint loop=0; loop<(GLuint)numtriangles; loop++) {        // loop through all the triangles
	glBegin(GL_TRIANGLES);		
	glNormal3f( 0.0f, 1.0f, 0.0f);
	
	x_m = sec.triangle[loop].vertex[0].x;
	y_m = sec.triangle[loop].vertex[0].y;
	z_m = sec.triangle[loop].vertex[0].z;
	u_m = sec.triangle[loop].vertex[0].u;
	v_m = sec.triangle[loop].vertex[0].v;
	glTexCoord2f(u_m,v_m); 
	glVertex3f(x_m,y_m,z_m);
	
	x_m = sec.triangle[loop].vertex[1].x;
	y_m = sec.triangle[loop].vertex[1].y;
	z_m = sec.triangle[loop].vertex[1].z;
	u_m = sec.triangle[loop].vertex[1].u;
	v_m = sec.triangle[loop].vertex[1].v;
	glTexCoord2f(u_m,v_m); 
	glVertex3f(x_m,y_m,z_m);
	
	x_m = sec.triangle[loop].vertex[2].x;
	y_m = sec.triangle[loop].vertex[2].y;
	z_m = sec.triangle[loop].vertex[2].z;
	u_m = sec.triangle[loop].vertex[2].u;
	v_m = sec.triangle[loop].vertex[2].v;
	glTexCoord2f(u_m,v_m); 
	glVertex3f(x_m,y_m,z_m);	
	
	glEnd();	
    }
    glEndList(); // tell the list is end now

}


}//namespace

