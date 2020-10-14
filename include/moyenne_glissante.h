/*****************************************************************************
      Moyenne glissante sur un certain nombre de valeurs
      Exemple d'utilisation :
  void loop() {

  int x, y, z;
  // Mettre des valeurs dans x, y, et z

  // Stocker les valeurs
  storeValues(x, y, z);

  // Calculer la moyenne
  Serial.print("\t");
  Serial.print(get_max_Z());
  Serial.print("\t");
  Serial.print(get_moy_z());
  // ...
  }

 ****************************************************************************/

#define NUM_VALUES 80

long dataX[NUM_VALUES];
long dataY[NUM_VALUES];
long dataZ[NUM_VALUES];

int indexData = 0;
long moyX = 0;
long maxX = 0;
long moyY = 0;
long maxY = 0;
long maxZ = 0;
long moyZ = 0;

void setup_moyenne_glissante() {
  moyX = 0;
  maxX = 0;
  moyY = 0;
  maxY = 0;
  moyZ = 0;
  maxZ = 0;
  // Init du tableau des moyennes glissantes
  for (indexData = 0; indexData < NUM_VALUES; indexData++) {
    dataX[indexData] = 0;
    dataY[indexData] = 0;
    dataZ[indexData] = 0;
  }
  indexData = 0;
}

void moyenneX() {
  // Caclul de la moyenne
  long somme = 0;
  maxX = 0;
  for (int v = 0; v < NUM_VALUES; v++) {
    somme += dataX[v];
    // Calcul du max
    if (dataX[v] > maxX) {
      maxX = dataX[v];
    }
  }
  moyX = somme / NUM_VALUES;
}

void moyenneY() {
  // Caclul de la moyenne
  long somme = 0;
  maxY = 0;
  for (int v = 0; v < NUM_VALUES; v++) {
    somme += dataY[v];
    // Calcul du max
    if (dataY[v] > maxY) {
      maxY = dataY[v];
    }
  }
  moyY = somme / NUM_VALUES;
}

void moyenneZ() {
  // Caclul de la moyenne
  long somme = 0;
  maxZ = 0;
  for (int v = 0; v < NUM_VALUES; v++) {
    somme += dataZ[v];
    // Calcul du max
    if (dataZ[v] > maxZ) {
      maxZ = dataZ[v];
    }
  }
  moyZ = somme / NUM_VALUES;
}

void storeValues(long valx, long valy, long valz) {
  dataX[indexData] = valx;
  dataY[indexData] = valy;
  dataZ[indexData] = valz;
  indexData++;

  if (indexData == NUM_VALUES) {
    indexData = 0;
  }
  moyenneX();
  moyenneY();
  moyenneZ();
}

long get_max_X() { return maxX; }

long get_moy_X() { return moyX; }

long get_max_Y() { return maxY; }

long get_moy_Y() { return moyY; }

long get_max_Z() { return maxZ; }

long get_moy_Z() { return moyZ; }
