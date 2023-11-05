//Anurag Amchi
//2022mcb1258
//Yash Vij
//2022mcb1284

//CS201 project 
//Efficient Collision detection using quadtrees

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

typedef struct Point
{ // Structure for point on the plane
    int x;
    int y;
} Point;

typedef struct Region
{ // Structure for the region on the plane
    Point *root;
    int dimension;
} Region;

typedef struct quadtree
{ // Structure for Quadtree
    Region *region;
    Point **points; // collection of pointers pointing to the points in a region
    // The four subregions the quadtree is subdivided into based on their direction with relative to centre
    struct quadtree *NE;
    struct quadtree *NW;
    struct quadtree *SW;
    struct quadtree *SE;
} quadtree;

Point *NewPoint(int x, int y)
{
    Point *point = (Point *)malloc(sizeof(Point));
    point->x = x;
    point->y = y;
    return point;
}

Region *NewRegion(Point *root, int dimension)
{ //  This makes the new region and then returns it
    Region *region = (Region *)malloc(sizeof(Region));
    region->root = root;
    region->dimension = dimension;
    return region;
}

quadtree *QuadNew(Region *root)
{
    quadtree *new = (quadtree *)malloc(sizeof(quadtree));
    new->NE = NULL;
    new->NW = NULL;
    new->SW = NULL;
    new->SE = NULL;
    new->region = root;
    new->points = (Point **)malloc(sizeof(Point *) * 4);
    int num = 0;
    while (num < 4)
    { // Assinging all the points to the NULL pointer
        new->points[num++] = NULL;
    }
    return new;
}

quadtree *subdivide(quadtree *tree)
{
    float temp = tree->region->dimension / 2;
    int dimension = ceil(temp);
    Point *toprightpoint = NewPoint(tree->region->root->x + dimension, tree->region->root->y + dimension);
    tree->NE = QuadNew(NewRegion(toprightpoint, dimension));
    Point *topleftpoint = NewPoint(tree->region->root->x - dimension, tree->region->root->y + dimension);
    tree->NW = QuadNew(NewRegion(topleftpoint, dimension));
    Point *bottomleftpoint = NewPoint(tree->region->root->x - dimension, tree->region->root->y - dimension);
    tree->SW = QuadNew(NewRegion(bottomleftpoint, dimension));
    Point *bottomrighttpoint = NewPoint(tree->region->root->x + dimension, tree->region->root->y - dimension);
    tree->SE = QuadNew(NewRegion(bottomrighttpoint, dimension));
    return tree;
}

bool inrange(Region *region, Point *point)
{ // This function checks if we insert the point in the region or not
    if (point->x > region->root->x + region->dimension)
        return 0;
    else if (point->y > region->root->y + region->dimension)
        return 0;
    else if (point->x < region->root->x - region->dimension)
        return 0;
    else if (point->y < region->root->y - region->dimension)
        return 0;
    else
        return 1;
}

int Size(Point *points[])
{ //  Returns the quadTree point size
    int num = 0;
    while (num < 4)
    {
        if (points[num] == NULL)
            return num;
        num++;
    }
    return 4;
}

bool insert(quadtree *tree, Point *point)
{ // Function to insert the point to the quadtree
    if (!(inrange(tree->region, point)))
    { // returns 0 if we cann't insert the point in the region
        // printf("Point is out of bounds of the current specified region\n");
        return 0;
    }
    long long Point_Size = Size(tree->points); // checks the size
    if (tree->NE == NULL && Point_Size < 4)
    {
        tree->points[Point_Size] = point;
        return 1;
    }
    if (tree->NE == NULL)
        tree = subdivide(tree);
    if (insert(tree->NE, point)) // INserts the point in the region
        return 1;
    else if (insert(tree->NW, point)) // INserts the point in the region
        return 1;
    else if (insert(tree->SW, point)) // INserts the point in the region
        return 1;
    else if (insert(tree->SE, point)) // INserts the point in the region
        return 1;
    else
        return 0;
}

bool collision(Region *one, Region *two) // This function
{
    if (one->root->x - one->dimension < two->root->x + two->dimension)
        return 1;
    else if (one->root->y - one->dimension < two->root->y + two->dimension)
        return 1;
    else if (one->root->x + one->dimension > two->root->x - two->dimension)
        return 1;
    else if (one->root->y + one->dimension > two->root->y - two->dimension)
        return 1;
    else
        return 0;
}

Point **Colliding_Points(quadtree *root, Region *range) // Gives the points that are inserted in the region
{
    Point **points;
    points = (Point **)malloc(sizeof(Point *) * (1 << 12));

    long long index = 0;

    int k = 0;
    while (k < (1 << 12)) // assigning all the points to the NULL
    {
        points[k] = NULL;
        k++;
    }

    if (collision(root->region, range) == 0) // If two regions are not intersecting then it will return the points
    {
        return points;
    }

    long long points_size = Size(root->points);

    k = 0;
    while (points_size > k) // inserting the point in the array if it present in the region
    {
        if (inrange(range, root->points[k]))
        {
            points[index++] = root->points[k];
        }
        k++;
    }

    if (root->NE == NULL)
    {
        return points;
    }

    Point **NE_range = Colliding_Points(root->NE, range); // inserting the point in the array if it present in the region
    for (int i = 0; i < 1024 && NE_range[i] != NULL; i++)
    {
        points[index++] = NE_range[i];
    }

    Point **NW_range = Colliding_Points(root->NW, range); // inserting the point in the array if it present in the region
    for (int i = 0; i < 1024 && NW_range[i] != NULL; i++)
    {
        points[index++] = NW_range[i];
    }

    Point **SW_range = Colliding_Points(root->SW, range); // inserting the point in the array if it present in the region
    for (int i = 0; i < 1024 && SW_range[i] != NULL; i++)
    {
        points[index++] = SW_range[i];
    }
    Point **SE_range = Colliding_Points(root->SE, range); // inserting the point in the array if it present in the region
    for (int i = 0; i < 1024 && SE_range[i] != NULL; i++)
    {
        points[index++] = SE_range[i];
    }

    return points;
}

void Print_point(struct Point *point){ // This prints the point and returns nothing
    printf("(%d, %d)\n", point->x, point->y);
}

int main()
{
    printf("Enter the centre of the QuadTree:\n");
    int x, y;
    printf("x:");
    scanf("%d", &x);
    printf("y:");
    scanf("%d", &y);
    printf("Centre is at (%d,%d)\n", x, y);
    int sidelength;
    printf("Enter the side length of the square region(<=50)\n");
    scanf("%d", &sidelength);
    int side = sidelength;
    float temp = (float)sidelength / 2;
    sidelength = ceil(temp) * 2;
    printf("Here is your working screen\n");
    Point *centre = NewPoint(x, y);
    Region *region = NewRegion(centre, ceil(temp));
    quadtree *tree = QuadNew(region);
    int num = 0;
    int arrsize = sidelength + 3;
    char screen[arrsize][arrsize];
    for (int i = 0; i < arrsize; i++)
    {
        screen[0][i] = '#';
        screen[arrsize - 1][i] = '#';
    }
    for (int i = 1; i < arrsize - 1; i++)
    {
        screen[i][0] = '#';
        screen[i][arrsize - 1] = '#';
    }
    for (int i = 1; i < arrsize - 1; i++)
    {
        for (int j = 1; j < arrsize - 1; j++)
        {
            screen[i][j] = '_';
        }
    }
    for (int i = 0; i < arrsize; i++)
    {
        for (int j = 0; j < arrsize; j++)
        {
            printf("%c ", screen[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    int relcentre = ceil((float)side / 2);
    relcentre += 1;
    printf("relcentre %d %d %d\n", relcentre, x, y);
    printf("Enter the number of points you want to Enter:");
    scanf("%d", &num);
    for (int i = 0; i < num; i++)
    {
        printf("Point %d:\n", i + 1);
        int a, b;
        printf("a:");
        scanf("%d", &a);
        printf("b:");
        scanf("%d", &b);
        Point *point = NewPoint(a, b);
        if (!inrange(region, point))
        {
            printf("Entered Point is out of bounds of the entered region\n");
            printf("Point not inserted\n");
            continue;
        }
        if (!insert(tree, point))
        {
            printf("Entered Point is out of bounds of the entered region\n");
            printf("Point not inserted\n");
        }
        else
        {
            screen[relcentre + y - b][relcentre - x + a] = 'O';
            printf("Point inserted\n");
        }
    }
    for (int i = 0; i < arrsize; i++)
    {
        for (int j = 0; j < arrsize; j++)
        {
            printf("%c ", screen[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    printf("Enter the centre of the object on which we want to detect collision\n");
    int objp, objq, objsidelength;
    printf("p:");
    scanf("%d", &objp);
    printf("q:");
    scanf("%d", &objq);
    printf("Enter side length of the square object:");
    scanf("%d", &objsidelength);
    temp = objsidelength / 2;
    objsidelength = ceil(temp);
    Point *objcen = NewPoint(objp, objq);
    Region *objregion = NewRegion(objcen, objsidelength);
    Point **res = Colliding_Points(tree, objregion); // forms the array of colliding points
    long long j = 0;
    if (res[j] == NULL)
        printf("No COlliding points.\n");
    else
    {
        printf("The colliding points are:\n");
        while (res[j] != NULL && j < (1 << 12)) // loop for printing the element in the given region
        {
            Print_point(res[j]);
            j++;
        }
    }

    return 0;
}
