//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***** Print ***** /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Print()
{

   Size_X = X_pos / ((float)MOTOR_X_STEP_PER_REV / SCREW_X * McSTEP_X);
   Size_Z = Z_pos / ((float)MOTOR_Z_STEP_PER_REV / SCREW_Z * McSTEP_Z);
   
   if      (err_1_flag == true)
   {
      lcd.setCursor(0, 0);
      lcd.print("BHИMAHИE:           ");
      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("    УПOPЫ HE 3AДAHЫ!"); 
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      return;   
   }
   else if (err_2_flag == true)
   {
      lcd.setCursor(0, 0);
      lcd.print("Установите суппорт  ");
      lcd.setCursor(0, 1);     
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print(" в исходную позицию!");
      lcd.setCursor(0, 3);
      lcd.print("                    ");      
      return;   
   }
   else if (Complete_flag == true)
   {
      lcd.setCursor(0, 0);
      lcd.print("                    ");
      lcd.setCursor(0, 1);     
      lcd.print("OПEPAЦИЯ 3ABEPШEHA! ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");       
      return;   
   }
   //================================    РЕЗЬБА   ===========================================================================================================================================

   if (Mode == Mode_Thread)
   { 
    if (SelectMenu == 0)
     {
      lcd.setCursor(0, 0);
      lcd.print("PE3ЬБA ");                                                   
      int lenTP = strlen(Thread_Info[Thread_Step].Thread_Print);
      if( lenTP == 4 )
      {
        lcd.setCursor(0, 1);       
        lcd.print("Шаг:          ");
        snprintf(LCD_Row_2, 5, "%s", Thread_Info[Thread_Step].Thread_Print);
        lcd.print(LCD_Row_2);
        lcd.print("мм");
      }
      else if(lenTP == 3 )
      {
        lcd.setCursor(0, 1);
        lcd.print("Шаг:          ");       
        snprintf(LCD_Row_2, 7, "%stpi", Thread_Info[Thread_Step].Thread_Print); 
        lcd.print(LCD_Row_2);
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Шаг:          ");        
        snprintf(LCD_Row_2, 7, "%s", Thread_Info[Thread_Step].Thread_Print);  
        lcd.print(LCD_Row_2);
      }
      switch(Sub_Mode_Thread)
      {
        case Sub_Mode_Thread_Int:
             lcd.setCursor(7, 0);
             lcd.print("   Bнyтpенняя"); 
             lcd.setCursor(0, 2);
             lcd.print("Пpoxoдов осталось:");
             snprintf(LCD_Row_4, 3, "%2d", (Thread_Info[Thread_Step].Pass - Pass_Nr+1 + PASS_FINISH + Pass_Fin) + Thr_Pass_Summ );
             lcd.print(LCD_Row_4);
             break;
        case Sub_Mode_Thread_Man:
             lcd.setCursor(7, 0);
             lcd.print("       Pyчная");
             lcd.setCursor(0, 2);
             lcd.print("Пpoxoдов всего:   ");
             snprintf(LCD_Row_4, 3, "%2d", (Thread_Info[Thread_Step].Pass + PASS_FINISH + Pass_Fin) + Thr_Pass_Summ );
             lcd.print(LCD_Row_4); 
             break;
        case Sub_Mode_Thread_Ext:
             lcd.setCursor(7, 0);
             lcd.print("     Hapyжняя");
             lcd.setCursor(0, 2);
             lcd.print("Пpoxoдов осталось:");
             snprintf(LCD_Row_4, 3, "%2d", (Thread_Info[Thread_Step].Pass - Pass_Nr+1 + PASS_FINISH + Pass_Fin) + Thr_Pass_Summ );
             lcd.print(LCD_Row_4);
             break;
      }
             lcd.setCursor(0, 3);
             lcd.print("Максимум,об/мин:"); 
             snprintf(LCD_Row_3, 5, "%s", Thread_Info[Thread_Step].Limit_Print); 
             lcd.print(LCD_Row_3);
     } 
     else if (SelectMenu == 1) 
     {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);                            
         lcd.print("Чистовых            "); 
         lcd.setCursor(0, 2); 
         lcd.print("Проходов:         ");
         snprintf(LCD_Row_3, 3, "%2d", PASS_FINISH + Pass_Fin);
         lcd.print(LCD_Row_3);
         lcd.setCursor(0, 3); 
         lcd.print("                    ");          
      }
     else if (SelectMenu == 2)    
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      }  
   }
   
   //================================    ПОДАЧА СИНХРОННАЯ   ===========================================================================================================================================
  
   else if (Mode == Mode_Feed)
   {  
    if (SelectMenu == 0)
     {
      lcd.setCursor(0, 0);
      lcd.print("СИНХРОННЫЙ ");
      lcd.setCursor(0, 1);      
      lcd.print("Подача,  мм/об: ");
      snprintf(LCD_Row_2, 5, "%1d.%02d", Feed_mm/100, Feed_mm%100);
      lcd.print(LCD_Row_2);            
      switch(Sub_Mode_Feed)
      {
        case Sub_Mode_Feed_Int:
             lcd.setCursor(11, 0);      
             lcd.print("  Внутр. ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);          
             break;
        case Sub_Mode_Feed_Man:
             lcd.setCursor(11, 0);      
             lcd.print("  Ручной ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов всего:   ");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
        case Sub_Mode_Feed_Ext:
             lcd.setCursor(11, 0);      
             lcd.print("  Наружн.");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
            }   
     }
     else if (SelectMenu == 1)  
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      }    
   }
   
   //================================    ПОДАЧА АСИНХРОННАЯ    ===========================================================================================================================================

   else if (Mode == Mode_aFeed)
   {   
    if (SelectMenu == 0) 
     {
      lcd.setCursor(0, 0);
      lcd.print("АСИНХРОННЫЙ ");
      lcd.setCursor(0, 1);
      lcd.print("Подача, мм/мин:  ");
      snprintf(LCD_Row_1, 4, "%3d", aFeed_mm);
      lcd.print(LCD_Row_1);
      //lcd.setCursor(0, 1);
      switch(Sub_Mode_aFeed)
      {
        case Sub_Mode_aFeed_Int:
             lcd.setCursor(12, 0);      
             lcd.print(" Внутр. ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на радиус: ");            
             break;
        case Sub_Mode_aFeed_Man:
             lcd.setCursor(12, 0);      
             lcd.print(" Ручной ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов всего:   ");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на радиус: ");              
             break;
        case Sub_Mode_aFeed_Ext:
             lcd.setCursor(12, 0);      
             lcd.print(" Наружн.");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на радиус: ");               
             break;
            } 
             if ( Ap == 5 ){
             snprintf(LCD_Row_4, 5, "%1d.%02d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);} 
             else {
             snprintf(LCD_Row_4, 5, " %1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);}
    }
        else if (SelectMenu == 1)
                          {
                           unsigned long Spindle_Angle = Enc_Pos * 360000 / ENC_TICK;
                           unsigned long Required_Angle = 360000 * (Current_Tooth - 1) / Total_Tooth;
                           lcd.setCursor(0, 0);
                           lcd.print("Текущий  угол:");
                           snprintf(LCD_Row_2, 7, "%3ld.%01ld", Spindle_Angle/1000, Spindle_Angle%1000/100);
                           lcd.print(LCD_Row_2);
                           lcd.print("\5");                           
                           lcd.setCursor(0, 1);
                           lcd.print("Делим круг на:");
                           snprintf(LCD_Row_1, 5,"%3d " , Total_Tooth);
                           lcd.print(LCD_Row_1);
                           lcd.print("\3\4");                           
                           lcd.setCursor(0, 2);
                           lcd.print("Выбор отметки:");
                           snprintf(LCD_Row_3, 5, "%3d ", Current_Tooth);
                           lcd.print(LCD_Row_3);
                           lcd.print("\1\2");
                           lcd.setCursor(0, 3);
                           lcd.print("Угол  сектора:");
                           snprintf(LCD_Row_4, 7, "%3ld.%01ld", Required_Angle/1000, Required_Angle%1000/100);
                           lcd.print(LCD_Row_4);
                           lcd.print("\5");
                          }
        else if (SelectMenu == 2) 
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      }  
                   
      
   }
   
   //================================    КОНУС ЛЕВЫЙ <    ===========================================================================================================================================

   else if (Mode == Mode_Cone_L)
   {
    if (SelectMenu == 0)
     {
      lcd.setCursor(0, 0);
      lcd.print("KOHУC <");
      snprintf(LCD_Row_1, 6, "%s ", Cone_Info[Cone_Step].Cone_Print);
      lcd.print(LCD_Row_1);
      lcd.setCursor(0, 1);
      lcd.print("Подача,  мм/об: ");
      snprintf(LCD_Row_2, 5, "%1d.%02d", Feed_mm/100, Feed_mm%100);
      lcd.print(LCD_Row_2);
      switch(Sub_Mode_Cone)
      {
        case Sub_Mode_Cone_Int:
             lcd.setCursor(11, 0);      
             lcd.print("  Внутр. ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);          
             break;
        case Sub_Mode_Cone_Man:
             lcd.setCursor(11, 0);      
             lcd.print("  Ручной ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов всего:   ");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
        case Sub_Mode_Cone_Ext:
             lcd.setCursor(11, 0);      
             lcd.print("  Наружн.");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
            }   
     }
     else if (SelectMenu == 1)
     {
      lcd.setCursor(0, 0);
      lcd.print("KOHУC <");
      snprintf(LCD_Row_1, 5, "%s", Cone_Info[Cone_Step].Cone_Print);
      lcd.print(LCD_Row_1);
      lcd.setCursor(11, 0);
      lcd.print("       \1\2");
      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
     }
     else if (SelectMenu == 2)
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      }   
   }
   
//   //================================    КОНУС ПРАВЫЙ >   ===========================================================================================================================================

   else if (Mode == Mode_Cone_R)
   {
    if (SelectMenu == 0)
     {
      lcd.setCursor(0, 0);
      lcd.print("KOHУC >");
      snprintf(LCD_Row_1, 6, "%s ", Cone_Info[Cone_Step].Cone_Print);
      lcd.print(LCD_Row_1);
      lcd.setCursor(0, 1);
      lcd.print("Подача,  мм/об: ");
      snprintf(LCD_Row_2, 5, "%1d.%02d", Feed_mm/100, Feed_mm%100);
      lcd.print(LCD_Row_2);
      switch(Sub_Mode_Cone)
      {
        case Sub_Mode_Cone_Int:
             lcd.setCursor(11, 0);      
             lcd.print("  Внутр. ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);          
             break;
        case Sub_Mode_Cone_Man:
             lcd.setCursor(11, 0);      
             lcd.print("  Ручной ");
             lcd.setCursor(0, 2);
             lcd.print("Проходов всего:   ");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
        case Sub_Mode_Cone_Ext:
             lcd.setCursor(11, 0);      
             lcd.print("  Наружн.");
             lcd.setCursor(0, 2);
             lcd.print("Проходов осталось:");
             snprintf(LCD_Row_3, 3, "%2d", Pass_Total-Pass_Nr+1);
             lcd.print(LCD_Row_3);
             lcd.setCursor(0, 3); 
             lcd.print("Съём на диаметр: ");
             snprintf(LCD_Row_4, 4, "%1d.%01d", Ap/100, Ap%100);
             lcd.print(LCD_Row_4);             
             break;
            }   
     }
     else if (SelectMenu == 1)
     {
      lcd.setCursor(0, 0);
      lcd.print("KOHУC >");
      snprintf(LCD_Row_1, 5, "%s", Cone_Info[Cone_Step].Cone_Print);
      lcd.print(LCD_Row_1);
      lcd.setCursor(11, 0);
      lcd.print("       \1\2");
      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
     }
     else if (SelectMenu == 2)
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      }   
   }
   
   //================================    РЕЖИМ ТАХОМЕТРА  ===========================================================================================================================================
   
   else if (Mode == Mode_Tachometr)  //////////////////////////////////////////////////////////////
   { 
    unsigned long Freq = ENC_LINE_PER_REV / (float)duration * Th;
       if (SelectMenu == 0)
        {
         unsigned long RPM = Freq * 60; 
         lcd.setCursor(0, 0);                            
         lcd.print("TAXOMETP            ");
         lcd.setCursor(0, 1);   
         lcd.print("         ");
         snprintf(LCD_Row_2, 5, "%4ld", RPM/10000); 
         lcd.print(LCD_Row_2); 
         lcd.print(" Oб/мин"); 
         lcd.setCursor(0, 2);
         lcd.print("                    ");
         lcd.setCursor(0, 3);
         lcd.print("                    ");       
         }
       else if (SelectMenu == 1)               
        { 
         lcd.setCursor(0, 0);                                 
         lcd.print("Чacтoтa вpaщeния    ");
         lcd.setCursor(0, 1);
         lcd.print("            ");
         snprintf(LCD_Row_2, 6, "%3ld.%01ld", Freq/10000, Freq%10);
         lcd.print(LCD_Row_2);   
         lcd.print(" Hz");  
         lcd.setCursor(0, 2);
         lcd.print("                    ");
         lcd.setCursor(0, 3);
         lcd.print("                    ");            
         }
         

   }
   
   //================================    РЕЖИМ ШАРА (Шароточка)   ===========================================================================================================================================

   else if (Mode == Mode_Sphere)
   { 
         
     if (SelectMenu == 0 && Sub_Mode_Sphere != Sub_Mode_Sphere_Int)
      {  
         lcd.setCursor(0, 0);
         lcd.print("ШAP \6");
         snprintf(LCD_Row_1, 5, "%2ld.%01ld", Sph_R_mm * 2 / 100, Sph_R_mm * 2 / 10 %10);
         lcd.print(LCD_Row_1);
         lcd.print("мм");
         lcd.setCursor(0, 1);
         lcd.setCursor(0, 1);
         lcd.print("Подача,  мм/об: ");
         snprintf(LCD_Row_2, 5, "%1d.%02d", Feed_mm/100, Feed_mm%100);
         lcd.print(LCD_Row_2);
          if (Sub_Mode_Sphere == Sub_Mode_Sphere_Man)
          {  
             lcd.setCursor(11, 0);
             lcd.print(" Oтключен");
          }
          else if (Sub_Mode_Sphere == Sub_Mode_Sphere_Ext)
          {  
             lcd.setCursor(11, 0);
             lcd.print("  Включен");
          }
          lcd.setCursor(0, 2);
          lcd.print("Оставить ножку \6"); 
          snprintf(LCD_Row_3, 5, "%ld.%02ld", Bar_R_mm*2/100, Bar_R_mm*2%100);
          lcd.print(LCD_Row_3);
          lcd.setCursor(0, 3);
          lcd.print("Проходов осталось");
          snprintf(LCD_Row_4, 4, "%3ld", Pass_Total_Sphr+2-Pass_Nr); 
          lcd.print(LCD_Row_4); 
      }
      else if (SelectMenu == 1)
      {  
         lcd.setCursor(0, 0);
         lcd.print("Шиpина peзцa: ");
         snprintf(LCD_Row_1, 5, "%1d.%02d", Cutter_Width/100, Cutter_Width%100);
         lcd.print(LCD_Row_1);
         lcd.print("мм");
         lcd.setCursor(0, 1);
         lcd.print("Шaг по ocи Z: ");
         snprintf(LCD_Row_2, 5, "%1d.%02d", Cutting_Width/100, Cutting_Width%100);
         lcd.print(LCD_Row_2);
         lcd.print("мм"); 
         lcd.setCursor(0, 2);
         lcd.print("                    ");
         lcd.setCursor(0, 3);
         lcd.print("                    ");
      }
     else if (SelectMenu == 2)
      {
         lcd.setCursor(0, 0);
         lcd.print("                    ");
         lcd.setCursor(0, 1);
         lcd.print("                    ");   
         lcd.setCursor(0, 2);
         if (Size_X%100 >= 0)
         lcd.print("Ocь X:      ");
         else 
         lcd.print("Ocь X:     -");
         snprintf(LCD_Row_1, 7, "%3ld.%02d", abs(Size_X/100), abs(Size_X%100));
         lcd.print(LCD_Row_1);
         lcd.print("мм");         
         lcd.setCursor(0, 3);
         if (Size_Z%100 >= 0)
         lcd.print("Ocь Z:      ");
         else 
         lcd.print("Ocь Z:     -");
         snprintf(LCD_Row_2, 7, "%3ld.%02d", abs(Size_Z/100), abs(Size_Z%100));
         lcd.print(LCD_Row_2); 
         lcd.print("мм");  
      } 
       if (Sub_Mode_Sphere == Sub_Mode_Sphere_Int)
         {
             lcd.setCursor(0, 0);
             lcd.print("                    ");
             lcd.setCursor(0, 1);     
             lcd.print(" Режим невозможен!  ");
             lcd.setCursor(0, 2);
             lcd.print("                    ");
             lcd.setCursor(0, 3);
             lcd.print("                    "); 
         }           
   }
   
   //================================    "Зарезервированный режим"   =======================================================================================================================================

   else if (Mode == Mode_Reserve)
   {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("              PE3EPB");
   }
}
