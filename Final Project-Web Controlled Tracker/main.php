<?php
  if(isset($_POST['command'])){
    if($_POST['command'] == 'auto'){
     $handle = fopen('info.txt', 'w');
     fwrite($handle, "Auto\n");
     fclose($handle);
      
    }else if($_POST['command']== 'manual'){
     // $handle = fopen('info.txt', 'w');
     // fwrite($handle, "Manual\n");
     //fclose($handle);
     //**** No longer Needed ****//
    }elseif($_POST['command'] == 'up'){
      echo "Moved Up..";
     $handle = fopen('info.txt', 'w');
     fwrite($handle, "Up\n");
     fclose($handle);
    
    }elseif($_POST['command'] == 'down'){
      echo "Moved Down...";
      $handle = fopen('info.txt', 'w');
      fwrite($handle, "Down\n");
      fclose($handle);
    

    }elseif($_POST['command'] == 'left'){
      echo "Moved Left...";
     $handle = fopen('info.txt', 'w');
     fwrite($handle, "Left\n");
     fclose($handle);
     

    }else{
      echo "Moved Right...";
      $handle = fopen('info.txt', 'w');
      fwrite($handle, "Right\n");
      fclose($handle);

    }
  }else{
    echo "ERROR SERVER RECEIVED NO COMMAND!";
  }

 ?>
